from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path 

#配置Isaac Sim自带的ROS 2，并重启一次让链接器读取库路径。
def ensure_isaac_ros_environment(): 
    """配置 Isaac Sim 自带的 ROS 2，并重启一次让链接器读取库路径。"""
    ros_lib = Path.home() / "isaacsim/exts/isaacsim.ros2.core/humble/lib"
    if not ros_lib.is_dir(): # 判断是否存在
        raise RuntimeError(f"Isaac Sim ROS 2 library directory not found: {ros_lib}")

    env = os.environ.copy()
    library_paths = [path for path in env.get("LD_LIBRARY_PATH", "").split(":") if path] # 获取LD_LIBRARY_PATH中的路径
    python_paths = [path for path in env.get("PYTHONPATH", "").split(":") if path] # 获取PYTHONPATH中的路径
    needs_restart = (
        env.get("ROS_DISTRO") != "humble"
        or env.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp"
        or str(ros_lib) not in library_paths # 判断ROS_LIB是否在LD_LIBRARY_PATH中
        or any(path.startswith("/opt/ros/") for path in python_paths) # 判断ROS的Python路径是否在PYTHONPATH中
    )

    if needs_restart:
        env["ROS_DISTRO"] = "humble" # 设置ROS_DISTRO为humble
        env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp" # 设置RMW_IMPLEMENTATION为rmw_fastrtps_cpp
        env["LD_LIBRARY_PATH"] = ":".join([*library_paths, str(ros_lib)]) # 将ROS_LIB添加到LD_LIBRARY_PATH中
        env["PYTHONPATH"] = ":".join(
            path for path in python_paths if not path.startswith("/opt/ros/") # 将ROS的Python路径排除在外
        )
        os.execve(sys.executable, [sys.executable, *sys.argv], env) # 重启进程


# 必须在导入并启动 SimulationApp 前执行。
ensure_isaac_ros_environment()

from isaacsim import SimulationApp
#获取参数
parser = argparse.ArgumentParser()
parser.add_argument("--stage")
parser.add_argument("--cube-prim", default="/World/Cube")
parser.add_argument("--robot-prim", default="/World/Franka")
parser.add_argument("--place-offset", nargs=3, type=float, default=(0.0, -0.30, 0.0))
parser.add_argument("--headless", action="store_true")
parser.add_argument("--joint-topic", default="/franka/joint_states") # 设置关节话题
ARGS = parser.parse_args()
#启动Isaacsim
SIMULATION_APP = SimulationApp(
    {
        "headless": ARGS.headless,
        "extra_args": ["--enable", "isaacsim.robot.manipulators.examples"],
    }
)
#调用工具库
import isaacsim.core.experimental.utils.app as app_utils
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

app_utils.enable_extension("isaacsim.ros2.bridge")
SIMULATION_APP.update()
import rclpy
from sensor_msgs.msg import JointState

CUBE_SIZE = 0.0515
EE_OFFSET = np.array([0.0, 0.005, 0.0])
ABOVE = 0.25
PHASE_STEPS = [180, 120, 90, 90, 180, 120, 90, 90]
PHASE_NAMES = (
    "Pre-grasp",
    "Approach",
    "Grasp",
    "Lift",
    "Transport",
    "Lower",
    "Release",
    "Retract",
)


class FrankaJointPublisher:
    #发布Franka手臂和夹爪关节作为sensor_msgs/JointState每个仿真步长

    def __init__(self, robot: Franka, topic: str):
        if not rclpy.ok(): # 判断是否初始化
            rclpy.init()
        self._robot = robot
        self._node = rclpy.create_node("franka_joint_publisher") # 创建节点
        self._pub = self._node.create_publisher(JointState, topic, 10) # 创建发布者 （10表示缓冲区大小）
        print(f"ROS 2 JointState -> {topic}", flush=True) # 打印关节话题

    def publish(self):
        pos = np.asarray(self._robot.get_joint_positions(), dtype=float).reshape(-1) # 获取关节位置 （-1表示自动计算维度）
        vel = np.asarray(self._robot.get_joint_velocities(), dtype=float).reshape(-1) # 获取关节速度 
        msg = JointState() # 创建关节状态消息
        msg.header.stamp = self._node.get_clock().now().to_msg() # 获取时间
        msg.name = [str(n) for n in self._robot.dof_names] # 获取关节名称
        msg.position = pos.tolist() # 转换为列表
        msg.velocity = vel.tolist() # 转换为列表
        self._pub.publish(msg) # 发布关节状态
        rclpy.spin_once(self._node, timeout_sec=0.0) # 等待一次


def ensure_pickable(prim_path: str):
    prim = get_prim_at_path(prim_path)
    if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        UsdPhysics.RigidBodyAPI.Apply(prim)
    kin = UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr()
    if kin and kin.Get():
        kin.Set(False)
    if not prim.HasAPI(UsdPhysics.CollisionAPI):
        UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.05)


def build_robot(world: World):
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


class FrankaPickPlace:
    def __init__(self, robot: Franka, cube_prim: str, place_offset: np.ndarray):
        self.robot = robot
        self.cube_prim = cube_prim
        self.place_offset = place_offset
        self.cube_position = np.zeros(3)
        self.target_position = np.zeros(3)
        self.grasp_ori = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))
        self.place_ori = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))
        self._event = 0
        self._step = 0
        self._warmup = 60
        self._rmp = RMPFlowController(name="franka_rmp", robot_articulation=robot)
        self._art = robot.get_articulation_controller()
        robot.gripper.set_action_deltas(None)
        for dof in robot.gripper.joint_dof_indicies:
            if dof is not None:
                self._art.switch_dof_control_mode(dof_index=dof, mode="position")

    def _ee_target(self):
        c, p, z = self.cube_position, self.target_position, self._event
        xy_c, xy_p = c[:2], p[:2]
        hi, lo = c[2] + ABOVE, c[2]
        phi, plo = p[2] + ABOVE, p[2]
        table = (
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

    def _read_cube(self):
        pos, quat = get_world_pose(self.cube_prim)
        self.cube_position = np.asarray(pos, dtype=float)
        self.target_position = self.cube_position + self.place_offset
        self.target_position[2] = max(self.cube_position[2], CUBE_SIZE * 0.5)
        yaw = float(quat_to_euler_angles(np.asarray(quat, dtype=float))[2])
        yaw -= (np.pi / 2.0) * np.round(yaw / (np.pi / 2.0))
        self.grasp_ori = euler_angles_to_quat(np.array([0.0, np.pi, yaw]))
        self.place_ori = euler_angles_to_quat(np.array([0.0, np.pi, 0.0]))
        print(f"pick={self.cube_position}  place={self.target_position}", flush=True)

    def _converged(self, goal: np.ndarray):
        if self._step < 40:
            return False
        fingers = self.robot.gripper.joint_dof_indicies
        widths = np.asarray(self.robot.get_joint_positions()[list(fingers)], dtype=float)
        if self._event == 2:
            return float(np.max(widths)) < 0.012
        if self._event == 6:
            return float(np.min(widths)) > 0.035
        ee = np.asarray(self.robot.end_effector.get_world_pose()[0], dtype=float)
        return float(np.linalg.norm(ee - goal)) < 0.025

    def forward(self):
        if self._event >= len(PHASE_STEPS):
            return False
        if self._warmup > 0:
            self._warmup -= 1
            if self._warmup == 0:
                print("warmup done, reading cube pose...", flush=True)
                self._read_cube()
                self._rmp.reset()
            return True
        goal = self._ee_target()
        if self._step == 0:
            print(f"  Phase {self._event}: {PHASE_NAMES[self._event]}", flush=True)
        if self._event in (2, 6):
            self._art.apply_action(
                self.robot.gripper.forward("close" if self._event == 2 else "open")
            )
        else:
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

    def reset(self):
        self._event = 0
        self._step = 0
        self._warmup = 60
        self._rmp.reset()
        self.robot.gripper.set_default_state(self.robot.gripper.joint_opened_positions)
        self.robot.gripper.post_reset()


def main():
    if ARGS.stage and not open_stage(ARGS.stage):
        raise RuntimeError(f"Could not open stage: {ARGS.stage}")
    world = World(stage_units_in_meters=get_stage_units() if ARGS.stage else 1.0)
    if not ARGS.stage:
        world.scene.add_default_ground_plane()
        if not is_prim_path_valid(ARGS.cube_prim):
            world.scene.add(
                DynamicCuboid(
                    name="pick_cube",
                    prim_path=ARGS.cube_prim,
                    position=np.array([0.40, 0.20, CUBE_SIZE * 0.5]),
                    orientation=euler_angles_to_quat(np.array([0.0, 0.0, np.deg2rad(35.0)])),
                    scale=np.full(3, CUBE_SIZE),
                    size=1.0,
                    color=np.array([0.1, 0.3, 0.9]),
                    mass=0.05,
                )
            )
    if not is_prim_path_valid(ARGS.cube_prim):
        raise RuntimeError(f"Missing cube prim: {ARGS.cube_prim}")
    ensure_pickable(ARGS.cube_prim)
    robot = build_robot(world)
    world.reset()
    task = FrankaPickPlace(robot, ARGS.cube_prim, np.asarray(ARGS.place_offset, dtype=float))
    joints = FrankaJointPublisher(robot, ARGS.joint_topic)
    need_reset = True
    done = False
    print("Stop→Play to re-run. Close window to exit.", flush=True)
    try:
        while SIMULATION_APP.is_running():
            world.step(render=not ARGS.headless)
            if world.is_stopped():
                need_reset = True
                done = False
                continue
            if not world.is_playing():
                continue
            joints.publish()
            if need_reset:
                world.reset()
                task.reset()
                need_reset = False
                done = False
                print("reset ok, warming up...", flush=True)
                continue
            if done:
                continue
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
    if rclpy.ok():
        rclpy.shutdown()
    SIMULATION_APP.close()
