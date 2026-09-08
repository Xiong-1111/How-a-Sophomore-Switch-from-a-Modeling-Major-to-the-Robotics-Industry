from __future__ import annotations

import argparse

from isaacsim import SimulationApp
#获取参数
parser = argparse.ArgumentParser()
parser.add_argument("--stage")
parser.add_argument("--cube-prim", default="/World/Cube")
parser.add_argument("--robot-prim", default="/World/Franka")
parser.add_argument("--place-offset", nargs=3, type=float, default=(0.0, -0.30, 0.0))
parser.add_argument("--headless", action="store_true")
ARGS = parser.parse_args()
#启动Isaacsim
SIMULATION_APP = SimulationApp(
    {
        "headless": ARGS.headless,
        "extra_args": ["--enable", "isaacsim.robot.manipulators.examples"],
    }
)
#调用工具库
import numpy as np 
from pxr import UsdPhysics #给方块加刚体、碰撞、质量
from isaacsim.core.api import World #负责往场景里加东西
from isaacsim.core.api.objects import DynamicCuboid #带物理的立方体
from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid #根据路径判断物体是否存在
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles #按方块旋转时用
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units, open_stage #打开场景
from isaacsim.core.utils.xforms import get_world_pose #读物体在世界里的位置和朝向
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers.rmpflow_controller import (
    RMPFlowController,
) #可以算出每个关节怎么转
from isaacsim.storage.native import get_assets_root_path

CUBE_SIZE = 0.0515 #cube的边长
EE_OFFSET = np.array([0.0, 0.005, 0.0]) #沿Y偏5mm，让爪夹中心对准方块
ABOVE = 0.25 #抬起时手比方块中心高25cm
PHASE_STEPS = [180, 120, 90, 90, 180, 120, 90, 90] # 8个阶段跑的帧数
PHASE_NAMES = ( #八个对应阶段，上方、接近、抓、抬、搬走、放下、放开、撤回
    "Pre-grasp",
    "Approach",
    "Grasp",
    "Lift",
    "Transport",
    "Lower",
    "Release",
    "Retract",
)


def ensure_pickable(prim_path: str):
    prim = get_prim_at_path(prim_path) #根据路径获取对应模型
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI): # 检查有没有物理刚体API
        UsdPhysics.RigidBodyAPI.Apply(prim) #加上刚体
    kin = UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr() #有没有kinematic
    if kin and kin.Get(): #如果有就解开
        kin.Set(False)
    if not prim.HasAPI(UsdPhysics.CollisionAPI): #如果没碰撞就加上
        UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.05) #质量为0.05


def build_robot(world: World):
    if not is_prim_path_valid(ARGS.robot_prim): #如果场景里没有机器人路径就从官方文件加载
        root = get_assets_root_path()
        if root is None:
            raise RuntimeError("Isaac assets root not found.")
        prim = add_reference_to_stage(
            root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd",
            ARGS.robot_prim,
        )
        prim.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger") #夹爪选AlternateFinger
    robot = world.scene.add(Franka(prim_path=ARGS.robot_prim, name="franka")) #把Franka放进world.scene
    robot.gripper.set_default_state(robot.gripper.joint_opened_positions) #爪夹初始默认张开
    return robot


#准备控制器
class FrankaPickPlace:

    def __init__(self, robot: Franka, cube_prim: str, place_offset: np.ndarray): 
        self.robot = robot
        self.cube_prim = cube_prim
        self.place_offset = place_offset
        self.cube_position = np.zeros(3) #抓取点放置点先填0
        self.target_position = np.zeros(3)
        self.grasp_ori = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))  # 抓：朝下
        self.place_ori = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))  # 放：朝下且 yaw=0（摆正）
        self._event = 0  # 当前的阶段
        self._step = 0  # 这一阶段走了多少帧
        self._warmup = 60  # 开始先等60帧让场景稳定
        self._rmp = RMPFlowController(name="franka_rmp", robot_articulation=robot)
        self._art = robot.get_articulation_controller()
        robot.gripper.set_action_deltas(None)
        for dof in robot.gripper.joint_dof_indicies:
            if dof is not None:
                self._art.switch_dof_control_mode(dof_index=dof, mode="position")
   #这一阶段爪要去哪
    def _ee_target(self):
        c, p, z = self.cube_position, self.target_position, self._event
        xy_c, xy_p = c[:2], p[:2] #取c和p的xy数值
        hi, lo = c[2] + ABOVE, c[2] #hi=方块上方25cm，lo=方块中心
        phi, plo = p[2] + ABOVE, p[2]
        table = (
            (*xy_c, hi), #方块上方
            (*xy_c, lo), #方块高度
            (*xy_c, lo), #合爪阶段，高度不变
            (*xy_c, hi), #抬起到方块上空
            (*xy_p, phi),#平移到目标点上方
            (*xy_p, plo),#下降到目标放置点
            (*xy_p, plo),#放置阶段，高度不变
            (*xy_p, phi),#撤离爪夹到目标点上方
        )
        return np.asarray(table[z], dtype=float) + EE_OFFSET#取出当前阶段的点加上5mm偏移量，修正机械臂手爪中心与方块中心对齐时的物理误差

    def _read_cube(self):
        pos, quat = get_world_pose(self.cube_prim) #读取方块的姿态
        self.cube_position = np.asarray(pos, dtype=float)
        self.target_position = self.cube_position + self.place_offset #目标点=方块位置+offset
        self.target_position[2] = max(self.cube_position[2], CUBE_SIZE * 0.5)#目标位置高度
        # 必须先算出 yaw，再用它生成抓取姿态
        yaw = float(quat_to_euler_angles(np.asarray(quat, dtype=float))[2])
        yaw -= (np.pi / 2.0) * np.round(yaw / (np.pi / 2.0))
        self.grasp_ori = euler_angles_to_quat(np.array([0.0, np.pi, yaw])) #抓，朝下，yaw方块
        self.place_ori = euler_angles_to_quat(np.array([0.0, np.pi, 0.0])) #放，朝下，yaw=0
        print(f"pick={self.cube_position}  place={self.target_position}", flush=True)

#看阶段是否完成
    def _converged(self, goal: np.ndarray): 
        if self._step < 40: #至少跑40帧才可以结束
            return False
        fingers = self.robot.gripper.joint_dof_indicies
        widths = np.asarray(self.robot.get_joint_positions()[list(fingers)], dtype=float)
        if self._event == 2: #在阶段2时，两指要小于0.012
            return float(np.max(widths)) < 0.012
        if self._event == 6: #在阶段6时，两指要大于0.035
            return float(np.min(widths)) > 0.035
        ee = np.asarray(self.robot.end_effector.get_world_pose()[0], dtype=float)#读右指的世界位置
        return float(np.linalg.norm(ee - goal)) < 0.025

    def forward(self):
        # 检查任务是否全部结束
        if self._event >= len(PHASE_STEPS):
            return False

        if self._warmup > 0:
            self._warmup -= 1
            if self._warmup == 0:
                # 场景稳定后再读方块位姿，并重置 RMPflow
                print("warmup done, reading cube pose...", flush=True)
                self._read_cube()
                self._rmp.reset()
            return True

        goal = self._ee_target() #某一阶段的第0帧打印阶段名
        if self._step == 0:
            print(f"  Phase {self._event}: {PHASE_NAMES[self._event]}", flush=True)

        if self._event in (2, 6): #阶段2和爪，阶段6张开
            self._art.apply_action(
                self.robot.gripper.forward("close" if self._event == 2 else "open")
            )
        else:
            # 0–3 用抓取姿态；4 起改用摆正姿态
            ori = self.place_ori if self._event >= 4 else self.grasp_ori
            self.robot.apply_action(
                self._rmp.forward(
                    target_end_effector_position=goal,
                    target_end_effector_orientation=ori,
                )
            )

        self._step += 1
        if self._converged(goal) or self._step >= PHASE_STEPS[self._event]:
            self._event += 1
            self._step = 0
        return True
#stop后再play全部回到初始状态
    def reset(self):
        self._event = 0
        self._step = 0
        self._warmup = 60
        self._rmp.reset()
        self.robot.gripper.set_default_state(self.robot.gripper.joint_opened_positions)
        self.robot.gripper.post_reset()

#主程序
def main():
    if ARGS.stage and not open_stage(ARGS.stage):
        raise RuntimeError(f"Could not open stage: {ARGS.stage}")
    #创建仿真世界
    world = World(stage_units_in_meters=get_stage_units() if ARGS.stage else 1.0)
    if not ARGS.stage:
        world.scene.add_default_ground_plane() #加地面
        if not is_prim_path_valid(ARGS.cube_prim):#新建一个方块
            world.scene.add(
                DynamicCuboid(
                    name="pick_cube", #名字
                    prim_path=ARGS.cube_prim,#路径
                    position=np.array([0.40, 0.20, CUBE_SIZE * 0.5]),#位置
                    orientation=euler_angles_to_quat(np.array([0.0, 0.0, np.deg2rad(35.0)])),#旋转35°
                    scale=np.full(3, CUBE_SIZE),
                    size=1.0,
                    color=np.array([0.1, 0.3, 0.9]),
                    mass=0.05,
                )
            )

    if not is_prim_path_valid(ARGS.cube_prim): 
        raise RuntimeError(f"Missing cube prim: {ARGS.cube_prim}")
    ensure_pickable(ARGS.cube_prim)
     #加载Franka
    robot = build_robot(world)
    world.reset()
    task = FrankaPickPlace(robot, ARGS.cube_prim, np.asarray(ARGS.place_offset, dtype=float))

    need_reset = True
    done = False #这一轮八个阶段是否结束
    print("Stop→Play to re-run. Close window to exit.", flush=True)

    try:
        while SIMULATION_APP.is_running(): #窗口还开这就一直循环
            world.step(render=not ARGS.headless) #有窗口就渲染

            if world.is_stopped():
                need_reset = True
                done = False
                continue
            if not world.is_playing():
                continue
	    #重置世界和任务
            if need_reset:
                world.reset()
                task.reset()
                need_reset = False
                done = False
                print("reset ok, warming up...", flush=True)
                continue

            if done:
                continue
            #调forward（）
            if not task.forward():
                done = True
                final = get_world_pose(ARGS.cube_prim)[0]
                print(
                    f"done. final={final} moved={np.linalg.norm(final - task.cube_position):.4f}",
                    flush=True,
                )
    except Exception:
        import traceback

        traceback.print_exc()
        raise


try:
    main()
finally:
    SIMULATION_APP.close()
