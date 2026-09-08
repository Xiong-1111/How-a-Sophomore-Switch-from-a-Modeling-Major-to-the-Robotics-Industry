
from __future__ import annotations

import argparse
import json
import time

parser = argparse.ArgumentParser()
parser.add_argument("--stage")
parser.add_argument("--cube-prim", default="/World/Cube")
parser.add_argument("--robot-prim", default="/World/Franka")
parser.add_argument("--place-offset", nargs=3, type=float, default=(0.0, -0.30, 0.0))
parser.add_argument("--headless", action="store_true")
parser.add_argument("--zmq-addr", default="tcp://127.0.0.1:5556") 
parser.add_argument(
    "--print",
    dest="print_joints", 
    action="store_true",
    help="Subscribe and print joints; do not start Isaac Sim.", 
)
ARGS = parser.parse_args()

# 打印关节信息
def print_joints_loop(addr: str):
    ctx = zmq.Context() # 创建一个上下文
    sock = ctx.socket(zmq.SUB) # 创建一个SUB类型的套接字（用来接受信息的接口）
    sock.setsockopt(zmq.SUBSCRIBE, b"") # 订阅所有消息（b""表示空字节串，表示订阅所有消息）
    sock.setsockopt(zmq.CONFLATE, 1) # 合并消息（1表示合并消息，0表示不合并消息）
    sock.connect(addr) # 连接到地址
    print(f"ZMQ SUB {addr}", flush=True) # 打印连接信息
    try:
        while True:
            data = json.loads(sock.recv_string()) # 接收消息
            names = data.get("name", []) # 获取关节名称
            pos = data.get("position", []) # 获取关节位置
            print("----", flush=True)
            for n, p in zip(names, pos):
                print(f"{n}: {p}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close() 
        ctx.term()


if ARGS.print_joints:
    print_joints_loop(ARGS.zmq_addr)
    raise SystemExit(0)

from isaacsim import SimulationApp

SIMULATION_APP = SimulationApp(
    {
        "headless": ARGS.headless,
        "extra_args": ["--enable", "isaacsim.robot.manipulators.examples"],
    }
)

import numpy as np
import zmq
from pxr import UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units, open_stage
from isaacsim.core.utils.xforms import get_world_pose
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers.rmpflow_controller import (
    RMPFlowController,
)
from isaacsim.storage.native import get_assets_root_path 

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


class FrankaJointZmqPublisher:

    def __init__(self, robot: Franka, addr: str):
        self._robot = robot
        self._ctx = zmq.Context() # 创建一个上下文
        self._sock = self._ctx.socket(zmq.PUB) # 创建一个PUB类型的套接字
        self._sock.setsockopt(zmq.SNDHWM, 1) # 设置发送缓冲区大小
        self._sock.bind(addr) # 绑定套接字到地址
        time.sleep(0.2) # 等待0.2秒
        print(f"ZMQ PUB {addr}", flush=True) # 打印连接信息

    def publish(self):
        pos = np.asarray(self._robot.get_joint_positions(), dtype=float).reshape(-1) # 获取关节位置
        vel = np.asarray(self._robot.get_joint_velocities(), dtype=float).reshape(-1) # 获取关节速度
        msg = {
            "name": [str(n) for n in self._robot.dof_names], # 获取关节名称
            "position": pos.tolist(), # 转换为列表
            "velocity": vel.tolist(), # 转换为列表  
            "time": time.time(), # 获取时间
        }
        self._sock.send_string(json.dumps(msg)) # 发送消息

    def close(self):
        self._sock.close() # 关闭套接字
        self._ctx.term() # 终止上下文


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
    joints = FrankaJointZmqPublisher(robot, ARGS.zmq_addr)
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
    finally:
        joints.close()


try:
    main()
finally:
    SIMULATION_APP.close()
