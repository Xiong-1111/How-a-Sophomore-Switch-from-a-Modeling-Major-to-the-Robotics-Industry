from __future__ import annotations

import argparse
import random

from isaacsim import SimulationApp

parser = argparse.ArgumentParser()
parser.add_argument("--cube-prim", default="/World/Cube")
parser.add_argument("--robot-prim", default="/World/Franka")
parser.add_argument("--conveyor-prim", default="/World/Conveyor")
parser.add_argument("--belt-speed", type=float, default=0.02)
parser.add_argument("--look-ahead", type=float, default=0.20)
parser.add_argument("--place-xy", nargs=2, type=float, default=(0.42, -0.35))
parser.add_argument("--cycles", type=int, default=5)
parser.add_argument("--headless", action="store_true")
ARGS = parser.parse_args()

SIMULATION_APP = SimulationApp(
    {
        "headless": ARGS.headless,
        "extra_args": [
            "--enable",
            "isaacsim.robot.manipulators.examples",
            "--enable",
            "isaacsim.asset.gen.conveyor",
            "--enable",
            "isaacsim.robot_motion.motion_generation",
        ],
    }
)

import numpy as np
from isaacsim.asset.gen.conveyor import create_conveyor_belt  # 导入生成传送带轨道的工具
from isaacsim.core.api import World
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid  # 导入检查物体在不在、把物体找出来的工具
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot_motion.motion_generation import (
    ArticulationKinematicsSolver,
    LulaKinematicsSolver,
)
from isaacsim.robot_motion.motion_generation.interface_config_loader import (
    load_supported_lula_kinematics_solver_config,
)
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf, PhysxSchema, UsdPhysics

CUBE_SIZE = 0.0515
EE_OFFSET = np.array([0.0, 0.005, 0.0])
ABOVE = 0.25
PLACE_ORI = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))

PHASE_STEPS = [99999, 150, 240, 90, 120, 320, 180, 100, 120]
PHASE_NAMES = (
    "Wait",
    "Pre-grasp",
    "Approach",
    "Grasp",
    "Lift",
    "Transport",
    "Lower",
    "Release",
    "Retract",
)

BELT_POS = np.array([0.50, 0.26, 0.02])  # 皮带中心
BELT_SCALE = np.array([1.20, 0.20, 0.04])  # 长 1.2m、宽 0.2m、厚 4cm
BELT_DIR = np.array([1.0, 0.0, 0.0])  # 传送带滚动的方向，沿+x
BELT_VEL = BELT_DIR * float(ARGS.belt_speed)  # 传送带的速度向量 = 方向 * 速度数值
BELT_SURFACE_Z = float(BELT_POS[2] + 0.5 * BELT_SCALE[2])  # 传送带距离地面的高度
CUBE_Z = BELT_SURFACE_Z + CUBE_SIZE * 0.5  # 方块中心高度
# 计算传送带最左端和最右端的 X 坐标范围
BELT_X_MIN = float(BELT_POS[0] - 0.5 * BELT_SCALE[0])
BELT_X_MAX = float(BELT_POS[0] + 0.5 * BELT_SCALE[0])
SPAWN_X = BELT_X_MIN + 0.18  # 刷出新方块的起点 X 坐标（传送带开头往里 18 厘米处）
SPAWN_Y_HALF = 0.05  # 刷出方块时，在 Y 轴方向左右随机晃动的幅度（前后 5 厘米）
PICK_X_MIN, PICK_X_MAX = 0.30, 0.58  # 机械臂能够抓取的范围
PLACE_XY = np.array(ARGS.place_xy, dtype=float)
PLACE_POS = np.array([PLACE_XY[0], PLACE_XY[1], CUBE_SIZE * 0.5], dtype=float)  # 地面放置点

# 官方 Franka Lula 配置里用的末端帧名
EE_FRAME = "right_gripper"


def ensure_pickable(prim_path: str) -> None:
    prim = get_prim_at_path(prim_path)
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim)
    kin = UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr()
    if kin and kin.Get():
        kin.Set(False)
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.05)


def build_robot(world: World) -> Franka:
    if not is_prim_path_valid(ARGS.robot_prim):
        root = get_assets_root_path()
        if root is None:
            raise RuntimeError("Isaac assets root not found.")
        prim = add_reference_to_stage(
            root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
            ARGS.robot_prim,
        )
        prim.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
    robot = world.scene.add(Franka(prim_path=ARGS.robot_prim, name="franka"))
    robot.gripper.set_default_state(robot.gripper.joint_opened_positions)
    return robot


# 创建传送带材质
def belt_material() -> PhysicsMaterial:
    return PhysicsMaterial(
        prim_path="/World/Physics_Materials/belt_mat",
        static_friction=1.0,
        dynamic_friction=1.0,
        restitution=0.0,
    )


# 创建方块材质
def cube_material() -> PhysicsMaterial:
    return PhysicsMaterial(
        prim_path="/World/Physics_Materials/cube_mat",
        static_friction=0.8,
        dynamic_friction=0.7,
        restitution=0.0,
    )


# 创建传送带
def build_conveyor(world: World):
    if not is_prim_path_valid(ARGS.conveyor_prim):
        world.scene.add(
            FixedCuboid(
                name="conveyor",
                prim_path=ARGS.conveyor_prim,
                position=BELT_POS,
                scale=BELT_SCALE,
                size=1.0,
                color=np.array([0.28, 0.28, 0.30]),
                physics_material=belt_material(),
            )
        )

    stage = get_current_stage()
    conveyor_prim = stage.GetPrimAtPath(ARGS.conveyor_prim)
    if not conveyor_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(conveyor_prim)
    UsdPhysics.RigidBodyAPI(conveyor_prim).CreateKinematicEnabledAttr(True)  # 传送带本身位置固定
    if not conveyor_prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(conveyor_prim)
    if not conveyor_prim.HasAPI(PhysxSchema.PhysxSurfaceVelocityAPI):
        PhysxSchema.PhysxSurfaceVelocityAPI.Apply(conveyor_prim)  # 加上表面速度 API

    node = create_conveyor_belt(stage, conveyor_prim, prim_name="ConveyorBeltGraph")
    node.GetAttribute("inputs:direction").Set(Gf.Vec3f(*BELT_DIR.tolist()))  # 设置传送方向
    node.GetAttribute("inputs:velocity").Set(float(ARGS.belt_speed))  # 设置传送速度
    graph_path = str(node.GetPath().GetParentPath())
    velocity_attr = stage.GetPrimAtPath(graph_path).GetAttribute("graph:variable:Velocity")
    velocity_attr.Set(float(ARGS.belt_speed))
    return conveyor_prim, velocity_attr, node


# 三处一起设速度，让传送带保持运转
def set_belt_running(conveyor_prim, velocity_attr, conveyor_node=None) -> None:
    speed = float(ARGS.belt_speed)
    if velocity_attr is not None:
        velocity_attr.Set(speed)
    if conveyor_node is not None and conveyor_node.IsValid():
        vel_in = conveyor_node.GetAttribute("inputs:velocity")
        if vel_in:
            vel_in.Set(speed)
        dir_in = conveyor_node.GetAttribute("inputs:direction")
        if dir_in:
            dir_in.Set(Gf.Vec3f(*BELT_DIR.tolist()))
    sv = PhysxSchema.PhysxSurfaceVelocityAPI(conveyor_prim)
    if sv:
        attr = sv.GetSurfaceVelocityAttr()
        if not attr:
            attr = sv.CreateSurfaceVelocityAttr()
        attr.Set(Gf.Vec3f(*(BELT_VEL.tolist())))


def random_spawn_pose() -> tuple[np.ndarray, np.ndarray]:
    y = BELT_POS[1] + random.uniform(-SPAWN_Y_HALF, SPAWN_Y_HALF)  # Y轴在皮带宽度内随机
    yaw = np.deg2rad(random.uniform(-25.0, 25.0))  # 偏转角随机转-25到+25°
    pos = np.array([SPAWN_X, y, CUBE_Z], dtype=float)  # 组合成三维位置
    quat = euler_angles_to_quat(np.array([0.0, 0.0, yaw]))  # 转换为四元数姿态
    return pos, quat


# 把方块传送到新位置
def spawn_cube(cube: DynamicCuboid, cycle: int) -> None:
    pos, quat = random_spawn_pose()  # 获取随机位置
    cube.set_world_pose(position=pos, orientation=quat)
    cube.set_linear_velocity(BELT_VEL)  # 给方块与皮带同向的初速度
    cube.set_angular_velocity(np.zeros(3))  # 旋转速度清零
    ensure_pickable(ARGS.cube_prim)  # 确保可抓取
    print(f"[cycle {cycle}] spawn cube at {pos}", flush=True)


class FrankaPickPlace:

    def __init__(self, robot: Franka, cube: DynamicCuboid):
        self.robot = robot
        self.cube = cube
        self.cube_position = np.zeros(3)
        self.picking_position = np.zeros(3)  # 预测去哪里拦截方块
        self._carry_position = np.zeros(3)  # 抓起方块时的坐标
        self.target_position = PLACE_POS.copy()
        self.grasp_ori = PLACE_ORI.copy()  # 抓取时的姿态
        self._event = 0
        self._step = 0
        self._warmup = 60
        self._ik_fail_count = 0

        # 给目标位姿，算出关节动作
        kinematics_config = load_supported_lula_kinematics_solver_config("Franka")
        if kinematics_config is None:
            raise RuntimeError("Failed to load Lula IK config for Franka.")
        lula = LulaKinematicsSolver(**kinematics_config)
        self._ik = ArticulationKinematicsSolver(robot, lula, EE_FRAME)

        self._art = robot.get_articulation_controller()  # 关节控制器
        robot.gripper.set_action_deltas(None)
        for dof in robot.gripper.joint_dof_indicies:
            if dof is not None:
                self._art.switch_dof_control_mode(dof_index=dof, mode="position")  # 爪夹改成位置模式

    def _read_cube(self) -> None:
        pos, quat = self.cube.get_world_pose()  # 读位置、姿态
        self.cube_position = np.asarray(pos, dtype=float)  # 计算拦截点
        # 算方块角度
        yaw = float(quat_to_euler_angles(np.asarray(quat, dtype=float))[2])
        yaw -= (np.pi / 2.0) * np.round(yaw / (np.pi / 2.0))
        self.grasp_ori = euler_angles_to_quat(np.array([0.0, np.pi, yaw]))

        # 阶段0-3，实施跟踪
        if self._event <= 3:
            if self._event <= 1:
                lead_s = float(ARGS.look_ahead)
            else:
                lead_s = 0.08
            self.picking_position = self.cube_position + BELT_VEL * lead_s  # 目标 = 方块位置 + 皮带速度 * 提前时间
            self.picking_position[2] = CUBE_Z

    # 判断方块是否进入抓取区
    def _in_pick_window(self) -> bool:
        x, y, z = self.cube_position
        return (
            PICK_X_MIN <= x <= PICK_X_MAX
            and abs(y - BELT_POS[1]) < 0.14  # y离皮带中心不超过14cm
            and abs(z - CUBE_Z) < 0.12
        )

    # 方块滑出传送带末端没抓到，主循环重生
    def cube_left_belt(self) -> bool:
        return float(self.cube_position[0]) > BELT_X_MAX - 0.06

    # 末端的世界坐标
    def _ee_pose(self) -> np.ndarray:
        return np.asarray(self.robot.end_effector.get_world_pose()[0], dtype=float)

    def _ee_target(self) -> np.ndarray:
        c = self.picking_position if self._event <= 3 else self._carry_position
        p, z = self.target_position, self._event
        xy_c, xy_p = c[:2], p[:2]
        hi, lo = c[2] + ABOVE, c[2]
        phi, plo = p[2] + ABOVE, p[2]
        table = (
            (*xy_c, hi),
            (*xy_c, hi),
            (*xy_c, lo),
            (*xy_c, lo),
            (*xy_c, hi),
            (*xy_p, phi),
            (*xy_p, plo),
            (*xy_p, plo),
            (*xy_p, phi),
        )
        return np.asarray(table[z], dtype=float) + EE_OFFSET

    # Lift之前用 grasp_ori（跟方块转），Lift起用固定朝下 PLACE_ORI
    def _target_orientation(self) -> np.ndarray:
        return PLACE_ORI if self._event >= 4 else self.grasp_ori

    # 两指开合宽度
    def _finger_widths(self) -> np.ndarray:
        fingers = self.robot.gripper.joint_dof_indicies
        return np.asarray(self.robot.get_joint_positions()[list(fingers)], dtype=float)

    # 末端XY离放置点是否小于4cm
    def _at_place_xy(self, tol: float = 0.04) -> bool:
        ee = self._ee_pose()
        return float(np.linalg.norm(ee[:2] - self.target_position[:2])) < tol

    # 下降阶段，XY差小于5cm且高度差小于4cm才算对准
    def _approach_close_enough(self) -> bool:
        ee = self._ee_pose()
        xy_err = float(np.linalg.norm(ee[:2] - self.cube_position[:2]))
        z_err = abs(float(ee[2]) - CUBE_Z)
        return xy_err < 0.05 and z_err < 0.04

    def _converged(self, goal: np.ndarray) -> bool:
        if self._event == 0:
            return self._in_pick_window() and self._step >= 10
        if self._step < 40:
            return False
        if self._event == 1:
            return float(np.linalg.norm(self._ee_pose() - goal)) < 0.05
        if self._event == 2:
            return self._approach_close_enough()
        if self._event == 3:
            return float(np.max(self._finger_widths())) < 0.012
        if self._event == 5:
            # 必须先到达悬停高度并到达XY指定位置 才能下降
            ee = self._ee_pose()
            xy_ok = self._at_place_xy(0.04)
            z_ok = abs(float(ee[2]) - (self.target_position[2] + ABOVE)) < 0.05
            return xy_ok and z_ok
        if self._event == 6:
            ee = self._ee_pose()
            xy_ok = self._at_place_xy(0.04)
            z_ok = abs(float(ee[2]) - self.target_position[2]) < 0.035
            return xy_ok and z_ok and self._step >= 60
        if self._event == 7:
            return float(np.min(self._finger_widths())) > 0.035
        return float(np.linalg.norm(self._ee_pose() - goal)) < 0.025

    # 调用 IK 求解器驱动机械臂向目标点移动（解不出则本帧不乱动）
    def _apply_arm(self, goal: np.ndarray) -> None:
        action, ok = self._ik.compute_inverse_kinematics( # 关节位置指令，通常只含臂不含手指
            target_position=np.asarray(goal, dtype=float),
            target_orientation=np.asarray(self._target_orientation(), dtype=float),
        )
        if ok:
            self.robot.apply_action(action)
            self._ik_fail_count = 0
        else:
            # ok=False：够不着，这一帧保持当前关节
            self._ik_fail_count += 1
            if self._ik_fail_count == 1 or self._ik_fail_count % 60 == 0:
                print(
                    f"  IK failed (x{self._ik_fail_count}) goal={goal} "
                    f"phase={PHASE_NAMES[self._event]}",
                    flush=True,
                )

    def _on_phase_enter(self) -> None:
        # 第四阶段，抬起时把方块位置冻成_carry_position（IK 无内部状态可重置）
        if self._event == 4:
            self._carry_position = self.cube_position.copy()
            self._carry_position[2] = CUBE_Z
            print(f"  carry from {self._carry_position}", flush=True)
        elif self._event == 5:
            print(f"  transport to place {self.target_position}", flush=True)
        # 第六阶段，如果XY还没到放置点，打警告
        elif self._event == 6:
            if not self._at_place_xy(0.06):
                print(
                    f"  warn: Lower starts but EE not at place "
                    f"(ee={self._ee_pose()[:2]}, place={self.target_position[:2]})",
                    flush=True,
                )

    def forward(self) -> bool:
        if self._event >= len(PHASE_STEPS):
            return False

        if self._warmup > 0:
            self._warmup -= 1
            if self._warmup == 0:
                print("warmup done, waiting for cube on belt...", flush=True)
                self._read_cube()
            return True

        # 0-3阶段，每帧读方块，如果方块沿皮带方向速度太慢，推一把（BELT_VEL）
        if self._event <= 3:
            self._read_cube()
            vel = np.asarray(self.cube.get_linear_velocity(), dtype=float)
            if float(np.dot(vel[:2], BELT_DIR[:2])) < 0.5 * float(ARGS.belt_speed):
                self.cube.set_linear_velocity(BELT_VEL)

        # 算这一帧爪夹该去的目标点
        goal = self._ee_target()
        if self._step == 0:
            print(f"  Phase {self._event}: {PHASE_NAMES[self._event]}", flush=True)
            if self._event == 1:
                print(f"pick={self.cube_position}  place={self.target_position}", flush=True)
            self._on_phase_enter()
            goal = self._ee_target()

        if self._event == 0 and self._step > 0 and self._step % 60 == 0:
            print(
                f"  Wait: cube_x={self.cube_position[0]:.3f} "
                f"in_window={self._in_pick_window()}",
                flush=True,
            )

        self._apply_arm(goal)  # 驱动机械臂移动
        if self._event == 3:
            self._art.apply_action(self.robot.gripper.forward("close"))  # 阶段三，合爪
        elif self._event == 7:
            self._art.apply_action(self.robot.gripper.forward("open"))  # 阶段七，张开爪夹

        self._step += 1
        timed_out = self._event != 0 and self._step >= PHASE_STEPS[self._event]  # 检查是否超时

        # 阶段2超时，够近或再多等180帧才进入下一阶段，避免空中就合爪
        if self._event == 2 and timed_out:
            ee = self._ee_pose()
            z_ok = abs(float(ee[2]) - CUBE_Z) < 0.07
            xy_ok = float(np.linalg.norm(ee[:2] - self.cube_position[:2])) < 0.08
            if (z_ok and xy_ok) or self._step >= PHASE_STEPS[2] + 180:
                self._event += 1
                self._step = 0
            return True

        # 不切阶段，继续追，避免在皮带上下放
        if self._event == 5 and timed_out and not self._at_place_xy(0.06):
            return True
        if self._event == 6 and timed_out:
            ee = self._ee_pose()
            z_ok = abs(float(ee[2]) - self.target_position[2]) < 0.05
            if not (self._at_place_xy(0.06) and z_ok):
                return True

        if self._converged(goal) or timed_out:
            self._event += 1
            self._step = 0
        return True

    def reset(self) -> None:
        self._event = 0
        self._step = 0
        self._warmup = 60
        self._ik_fail_count = 0
        self.robot.gripper.set_default_state(self.robot.gripper.joint_opened_positions)
        self.robot.gripper.post_reset()


def main() -> None:
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    world.get_physics_context().enable_gpu_dynamics(False)

    set_camera_view(eye=np.array([1.6, -1.4, 1.2]), target=np.array([0.4, 0.1, 0.2]))

    print(
        f"[IK] belt={ARGS.belt_speed} m/s  place={tuple(ARGS.place_xy)}  cycles={ARGS.cycles}",
        flush=True,
    )

    conveyor_prim, velocity_attr, conveyor_node = build_conveyor(world)  # 造传送带
    # 生成随机位姿的蓝色方块
    pos0, quat0 = random_spawn_pose()
    cube = world.scene.add(
        DynamicCuboid(
            name="pick_cube",
            prim_path=ARGS.cube_prim,
            position=pos0,
            orientation=quat0,
            scale=np.full(3, CUBE_SIZE),
            size=1.0,
            color=np.array([0.1, 0.3, 0.9]),
            mass=0.05,
            physics_material=cube_material(),
            linear_velocity=BELT_VEL,
        )
    )
    ensure_pickable(ARGS.cube_prim)
    robot = build_robot(world)

    world.reset()  # 初始化整个世界物理状态
    world.get_physics_context().enable_gpu_dynamics(False)
    set_belt_running(conveyor_prim, velocity_attr, conveyor_node)

    task = FrankaPickPlace(robot, cube)
    need_reset = True
    cycle = 1
    idle = False
    print("Stop→Play to re-run. Close window to exit.", flush=True)

    try:
        while SIMULATION_APP.is_running():
            world.step(render=not ARGS.headless)
            # 按了stop，需要重置然后跳过
            if world.is_stopped():
                need_reset = True
                idle = False
                continue
            if not world.is_playing():
                continue

            # 确保传送带一直在跑
            set_belt_running(conveyor_prim, velocity_attr, conveyor_node)

            # 需要重置、出第一块方块
            if need_reset:
                world.reset()
                world.get_physics_context().enable_gpu_dynamics(False)
                set_belt_running(conveyor_prim, velocity_attr, conveyor_node)
                cycle = 1
                idle = False
                spawn_cube(cube, cycle)  # 重新刷新方块
                task.reset()
                need_reset = False
                print("reset ok, warming up...", flush=True)
                continue

            # 五块都抓完，空转等重新开始
            if idle:
                continue

            if task._event == 0 and task._warmup == 0:
                task._read_cube()
                if task.cube_left_belt():  # 如果方块滑出传送带末端，重新刷新一个方块
                    print("cube left the belt, respawning...", flush=True)
                    spawn_cube(cube, cycle)
                    task.reset()
                    continue

            if not task.forward():  # 如果返回False说明方块抓完并放好了
                final = np.asarray(cube.get_world_pose()[0], dtype=float)
                print(f"[cycle {cycle}/{ARGS.cycles}] placed at {final}", flush=True)
                if cycle >= int(ARGS.cycles):
                    print("all cycles done. Stop→Play to restart.", flush=True)
                    idle = True
                    continue
                cycle += 1
                spawn_cube(cube, cycle)
                task.reset()
    except Exception:
        import traceback

        traceback.print_exc()
        raise


try:
    main()
finally:
    SIMULATION_APP.close()
