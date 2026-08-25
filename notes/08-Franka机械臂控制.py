```python
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.robot.manipulators.examples")

import numpy as np
import omni.timeline
from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

from isaacsim.core.api import World
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid
from isaacsim.core.utils.xforms import get_world_pose
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import (
    PickPlaceController,
)

ROBOT_PRIM = "/World/Franka"
CUBE_PRIM = "/World/Cube"
PLACE_OFFSET = np.array([0.0, -0.30, 0.0])  # 放置点相对抓取点偏移（Y 负方向 30cm）
TARGET_CUBE_SIZE = 0.0515
END_EFFECTOR_OFFSET = np.array([0.0, 0.005, 0.0])  # 末端在右指上，沿 Y 挪 5mm，让两指包住方块
EVENTS_DT = [0.008, 0.005, 0.02, 0.02, 0.02, 0.008, 0.0025, 0.05, 0.008, 0.08]
# 控制器 10 个阶段每步推进量；数字越小该阶段越久


def get_world_aabb_size(prim_path: str) -> np.ndarray:
    """获取方块的世界坐标包围盒尺寸。"""
    prim = get_prim_at_path(prim_path)
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
    )
    world_range = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    return np.asarray(world_range.GetSize(), dtype=float)


def prepare_pick_cube(prim_path: str) -> np.ndarray:
    if not is_prim_path_valid(prim_path):
        raise RuntimeError(f"Cube prim does not exist: {prim_path}")

    prim = get_prim_at_path(prim_path)

    # 添加刚体和碰撞
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim)
        print(f"Applied RigidBodyAPI to {prim_path}")
    rigid_body = UsdPhysics.RigidBodyAPI(prim)
    kinematic_attr = rigid_body.GetKinematicEnabledAttr()
    if kinematic_attr and kinematic_attr.Get():
        kinematic_attr.Set(False)
        print(f"{prim_path} was kinematic; switched to dynamic.")
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)
        print(f"Applied CollisionAPI to {prim_path}")
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.05)

    # 添加摩擦力
    stage = prim.GetStage()
    material_path = prim.GetPath().AppendChild("PickFriction")
    if not stage.GetPrimAtPath(material_path):
        UsdShade.Material.Define(stage, material_path)
        material_api = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(material_path))
        material_api.CreateStaticFrictionAttr(1.5)
        material_api.CreateDynamicFrictionAttr(1.5)
        material_api.CreateRestitutionAttr(0.0)  # 弹性为 0
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        UsdShade.Material(stage.GetPrimAtPath(material_path))
    )

    size = get_world_aabb_size(prim_path)
    current = float(np.max(size))
    if current > 0.08 or current < 0.025:
        factor = TARGET_CUBE_SIZE / max(current, 1e-6)
        xformable = UsdGeom.Xformable(prim)
        scale_ops = [
            op
            for op in xformable.GetOrderedXformOps()
            if op.GetOpType() == UsdGeom.XformOp.TypeScale
        ]
        if scale_ops:
            old = np.array(scale_ops[0].Get() or (1.0, 1.0, 1.0), dtype=float)
            scale_ops[0].Set(Gf.Vec3d(*(old * factor)))
        else:
            xformable.AddScaleOp().Set(Gf.Vec3d(factor, factor, factor))
        size = get_world_aabb_size(prim_path)
        print(f"Rescaled {prim_path} to AABB {size} (Franka fingers only open ~8 cm).")

    # 设置方块位置和姿态
    xform = SingleXFormPrim(prim_path, reset_xform_properties=False)
    position, orientation = xform.get_world_pose()
    position = np.asarray(position, dtype=float)
    position[2] = float(size[2]) * 0.5 + 0.001
    reach = float(np.linalg.norm(position[:2]))
    if reach > 0.75 or reach < 0.20:
        print(
            f"Cube XY reach {reach:.3f} m is outside a reliable workspace; moving to (0.40, 0.20)."
        )
        position[0], position[1] = 0.40, 0.20  # 超出可靠工作空间则移到 (0.40, 0.20)
    xform.set_world_pose(position, orientation)
    return get_world_aabb_size(prim_path)


def get_or_create_robot(world: World) -> Franka:
    """获取或创建机器人。"""
    robot_prim = get_prim_at_path(ROBOT_PRIM)
    gripper_set = robot_prim.GetVariantSet("Gripper")
    if gripper_set and gripper_set.IsValid():
        gripper_set.SetVariantSelection("AlternateFinger")

    if world.scene.object_exists("franka"):
        robot = world.scene.get_object("franka")
    else:
        robot = world.scene.add(Franka(prim_path=ROBOT_PRIM, name="franka"))
        robot.gripper.set_default_state(robot.gripper.joint_opened_positions)

    robot.initialize()
    robot.gripper.set_action_deltas(None)
    for dof_index in robot.gripper.joint_dof_indicies:
        if dof_index is not None:
            robot.get_articulation_controller().switch_dof_control_mode(
                dof_index=dof_index, mode="position"
            )
    return robot


# 检查机器人是否存在
if not is_prim_path_valid(ROBOT_PRIM):
    raise RuntimeError(f"Robot prim does not exist: {ROBOT_PRIM}")

# 准备方块：大小与位置
cube_size = prepare_pick_cube(CUBE_PRIM)

# 未 Play 则自动 Play
timeline = omni.timeline.get_timeline_interface()
if not timeline.is_playing():
    timeline.play()

# 复用已有 World
world = World.instance()
if world is None:
    world = World(stage_units_in_meters=1.0)

world.reset()
robot = get_or_create_robot(world)

controller = PickPlaceController(
    name="franka_pick_place",
    gripper=robot.gripper,
    robot_articulation=robot,
    events_dt=EVENTS_DT,
)
articulation_controller = robot.get_articulation_controller()

# 记录方块初始位置与放置点
initial_cube_position, _ = get_world_pose(CUBE_PRIM)
initial_cube_position = np.asarray(initial_cube_position, dtype=float).copy()
place_position = initial_cube_position + PLACE_OFFSET
place_position[2] = float(cube_size[2]) * 0.5

print("Cube prim:", CUBE_PRIM)
print("Cube position:", initial_cube_position)
print("Cube AABB size:", cube_size)
print("Place position:", place_position)
print("Keep the viewport timeline Playing.")

# 取消已注册的回调
if "pick_place_cb_id" in globals() and pick_place_cb_id is not None:
    try:
        SimulationManager.deregister_callback(pick_place_cb_id)
    except Exception:
        pass
    pick_place_cb_id = None


def on_physics_step(dt, context=None):
    global pick_place_cb_id
    if controller.is_done():
        final_pos, _ = get_world_pose(CUBE_PRIM)
        moved = float(np.linalg.norm(final_pos - initial_cube_position))
        print("Pick-and-place controller finished.")
        print("Final cube position:", final_pos)
        print("Moved distance:", moved)
        if moved < 0.05:
            print(
                "WARNING: cube barely moved. Check that AABB printed above is ~0.05 m, "
                "the cube is dynamic (not kinematic), and Play stayed on."
            )
        try:
            SimulationManager.deregister_callback(pick_place_cb_id)
        except Exception:
            pass
        pick_place_cb_id = None
        return

    action = controller.forward(
        picking_position=initial_cube_position,
        placing_position=place_position,
        current_joint_positions=robot.get_joint_positions(),
        end_effector_offset=END_EFFECTOR_OFFSET,
    )
    articulation_controller.apply_action(action)


pick_place_cb_id = SimulationManager.register_callback(
    on_physics_step, IsaacEvents.POST_PHYSICS_STEP
)
print("Pick-place callback registered.")
```
