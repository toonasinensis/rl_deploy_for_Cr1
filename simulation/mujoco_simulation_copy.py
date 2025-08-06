"""
 * @file mujoco_simulation.py
 * @brief simulation in mujoco
 * @author mayuxuan
 * @version 1.0
 * @date 2025-05-08
 *
 * @copyright Copyright (c) 2024  DeepRobotics
"""

import os
import time
import socket
import struct
import threading
from pathlib import Path
from scipy.spatial.transform import Rotation
import numpy as np
import mujoco
import mujoco.viewer

MODEL_NAME = "CR01B-pro"
XML_PATH = "urdf_model/CR01B-pro/B-20250522.xml"
LOCAL_PORT = 20001
CTRL_IP = "127.0.0.1"
CTRL_PORT = 30010
USE_VIEWER = True
DT = 0.001
RENDER_INTERVAL = 45

URDF_INIT = {
    "CR01B-pro": np.array([0, ] * 29, dtype=np.float32)
}

from multiprocessing import shared_memory
import numpy as np
 


class MuJoCoSimulation:
    def __init__(self,
                 model_key: str = MODEL_NAME,
                 xml_relpath: str = XML_PATH,
                 local_port: int = LOCAL_PORT,
                 ctrl_ip: str = CTRL_IP,
                 ctrl_port: int = CTRL_PORT):

        # UDP 通信
        self.recive_bool = True
        self.local_port = local_port
        self.ctrl_addr = (ctrl_ip, ctrl_port)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(("127.0.0.1", local_port))
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 加载 MJCF
        xml_full = str(Path(__file__).resolve().parent / xml_relpath)
        if not os.path.isfile(xml_full):
            raise FileNotFoundError(f"Cannot find MJCF: {xml_full}")

        self.model = mujoco.MjModel.from_xml_path(xml_full)
        self.model.opt.timestep = DT
        self.data = mujoco.MjData(self.model)

        # 机器人自由度列表
        self.actuator_ids = [a for a in range(self.model.nu)]  # 0..11
        self.dof_num = len(self.actuator_ids)
        # print("self.dof_num,",self.dof_num)
        print("model_key", model_key)
        # 初始化站立姿态
        self._set_initial_pose(model_key)
        self.pyload_old = None
        # 缓存
        self.kp_cmd = np.zeros((self.dof_num, 1), np.float32)
        self.kd_cmd = np.zeros_like(self.kp_cmd)
        self.pos_cmd = np.zeros_like(self.kp_cmd)
        self.vel_cmd = np.zeros_like(self.kp_cmd)
        self.tau_ff = np.zeros_like(self.kp_cmd)
        self.input_tq = np.zeros_like(self.kp_cmd)

        # IMU
        self.last_base_linvel = np.zeros((3, 1), np.float64)
        self.timestamp = 0.0
        self.timestamp_last = 0

        print(f"[INFO] MuJoCo model loaded, dof = {self.dof_num}")

        # 可视化
        self.viewer = None
        if USE_VIEWER:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)


        # self.shm_phase = shared_memory.SharedMemory(name="phase")  # 连接已有共享内存//确保他们的生存周期为整个进程，防止被回收
        # self.shm_pos = shared_memory.SharedMemory(name="init_pos")  # 连接已有共享内存
        # self.shm_quat = shared_memory.SharedMemory(name="init_quat")  # 连接已有共享内存
        # self.shm_all_joint_mj = shared_memory.SharedMemory(name="all_joint_mj")  # 连接已有共享内存

        # 用numpy映射共享内存
        # self.arr_phase = np.ndarray((1,), dtype=np.float32, buffer=self.shm_phase.buf)
        # self.arr_pos = np.ndarray((3,), dtype=np.float32, buffer=self.shm_pos.buf)
        # self.arr_quat = np.ndarray((4,), dtype=np.float32, buffer=self.shm_quat.buf)
        # self.arr_init_dof = np.ndarray((self.dof_num,), dtype=np.float32, buffer=self.shm_all_joint_mj.buf)
        # dof_list = [0.0054, 0.0491, 0.2342, -0.1076, 0.5344, -0.4301, 1.2467, 0.0, 0.0,
        #         0.0, -0.1189, -0.5167, 0.2878, 1.2303, 0.0, 0.0, 0.0, -0.1114,
        #         0.0076, 0.147, 0.2569, -0.0637, 0.0656, -0.1536, -0.0466, -0.1655, 0.325,
        #         -0.0538, -0.0304]
        dof_list = [
            -0.014, 0.049, 0.002, -0.246, 0.161, -0.328, 1.411, 0.0, 0.0, 0.0,
            -0.328, -0.137, 0.502, 1.441, 0.0, 0.0, 0.0, -0.063, -0.057, -0.019,
            0.008, 0.079, 0.058, -0.144, 0.037, -0.168, 0.186, 0.04, 0.09
        ]

        self.arr_init_dof = np.array(dof_list, dtype=np.float64)
        # self.arr_pos = np.array([-0.0075 ,-0.0068 , 0.8614], dtype=np.float64)
        # self.arr_quat = np.array([ 0.7111 , 0.015,  -0.0019 , 0.703 ], dtype=np.float64)
        self.arr_pos = np.array( [-0.129 , 0.507 , 0.857], dtype=np.float64)
        self.arr_quat = np.array([0.737, -0.017 , 0.018 , 0.676], dtype=np.float64)

        # print(self.arr_init_dof)
        # import ipdb;ipdb.set_trace();
        # print(arr)  # 共享内存中的数据
    # --------------------------------------------------------

    def _set_initial_pose(self, key: str):
        """关节位置设置为与 PyBullet 脚本一致的初始角度"""
        qpos0 = self.data.qpos.copy()
        print("qpos0",qpos0.shape)
        print(" self.dof_num",self.dof_num, qpos0[7:7 + self.dof_num].shape)
        # qpos0[7:7 + self.dof_num] = URDF_INIT[key]  # ,3-6 basequat，0-2 basepos
        qpos0[:self.dof_num] = URDF_INIT[key]  # ,3-6 basequat，0-2 basepos

        # qpos0[:3] = np.array([0, 0, 0.5])
        # qpos0[3:7] = np.array([0.707, 0, -0.707, 0])
        self.data.qpos[:] = qpos0
        mujoco.mj_forward(self.model, self.data)

        # ---------------- 主循环 -----------------

    def start(self):
        # 接收线程
        threading.Thread(target=self._udp_receiver, daemon=True).start()
        print(f"[INFO] UDP receiver on 127.0.0.1:{self.local_port}")

        # 主模拟循环
        step = 0
        last_time = time.time()
        init_time = time.time()
        while True:
            if time.time() - last_time >= DT:
                last_time = time.time()
                step += 1
                # 控制律
                # print("self.arr_phase[0]", self.arr_phase[0])
                # if self.tau_ff[0] > 88   :
                self._apply_joint_torque()
                # else:
                #     # self._apply_joint_torque()
                #     self._set_mimic_init_state()
                # 模拟一步
                mujoco.mj_step(self.model, self.data)
                
                self.timestamp = time.time() - init_time
                
                # 采样 & 发送观测
                self._send_robot_state(step)
                # 
                # 可视化
                if self.viewer and step % RENDER_INTERVAL == 0:
                    t1 = time.time()
                    self.viewer.sync()
                    t2 = time.time()
                    # print(t2-t1)
                #
                # 
             # dt = time.perf_counter() - t0
            # if dt < DT:
            #     time.sleep(DT - dt)

    # --------------------------------------------------------

    def _udp_receiver(self):
        """
        12f kp | 12f pos | 12f kd | 12f vel | 12f tau  = 240 bytes
        """
        # fmt = "12f" * 5
        fmt = f'{self.dof_num}f' * 5
        expected = struct.calcsize(fmt)  # 240
        while True:
            data, addr = self.recv_sock.recvfrom(expected)
            if len(data) < expected:
                print(f"[WARN] UDP packet size {len(data)} != {expected}")
                continue
            unpacked = struct.unpack(fmt, data)
            self.kp_cmd = np.asarray(unpacked[0:self.dof_num], dtype=np.float32).reshape(self.dof_num, 1)
            self.pos_cmd = np.asarray(unpacked[self.dof_num:self.dof_num * 2], dtype=np.float32).reshape(self.dof_num,
                                                                                                         1)
            self.kd_cmd = np.asarray(unpacked[self.dof_num * 2:self.dof_num * 3], dtype=np.float32).reshape(
                self.dof_num, 1)
            self.vel_cmd = np.asarray(unpacked[self.dof_num * 3:self.dof_num * 4], dtype=np.float32).reshape(
                self.dof_num, 1)
            self.tau_ff = np.asarray(unpacked[self.dof_num * 4:], dtype=np.float32).reshape(self.dof_num, 1)
            # print(f"[UDP] cmd from {addr}")
            # print("kp",self.kp_cmd.transpose())
            # print("kd",self.kd_cmd.transpose())
            # print("pos_cmd",self.pos_cmd.transpose())
            # print("vel_cmd",self.vel_cmd.transpose())

    # --------------------------------------------------------

    def _apply_joint_torque(self):
        # 当前关节状态#TODO:
        q = self.data.qpos[ :self.dof_num].reshape(-1, 1)
        dq = self.data.qvel[ : self.dof_num].reshape(-1, 1)

        # τ = kp*(q_d - q) + kd*(dq_d - dq) + τ_ff
        self.input_tq = (
                self.kp_cmd * (self.pos_cmd - q) +
                self.kd_cmd * (self.vel_cmd - dq) #+
               # self.tau_ff
        )
        # # 调试
        print("=== [Joint Command Debug] ===")
        print(f"[Target Pos]:\n{self.pos_cmd.T}")
        print(f"[Actual Pos]:\n{q.T}")
        print(f"[Target Vel]:\n{self.vel_cmd.T}")
        print(f"[Actual Vel]:\n{dq.T}")
        print(f"[Kp Term]:\n{self.kp_cmd.T}")
        print(f"[Kd Term]:\n{self.kd_cmd.T}")
        print(f"[Feedforward Tau]:\n{self.tau_ff.T}")
        print(f"[Final Torque Output]:\n{self.input_tq.T}")

        # 写入 control 缓冲区
        self.data.ctrl[:] = self.input_tq.flatten()

    def _set_mimic_init_state(self):
        # import ipdb;ipdb.set_trace()
        self.data.qpos[-self.dof_num:] = self.arr_init_dof.astype(np.float64)
        # print( "self.arr_init_",self.arr_init_dof.astype(np.float64))
        # print( "self.qpos",self.arr_pos.astype(np.float64))
        # print( "self.quat",self.arr_quat.astype(np.float64))

        # self.data.qpos[0:3] = self.arr_pos.astype(np.float64)
        # self.data.qpos[3:7] = self.arr_quat.astype(np.float64)
        self.data.qvel[:] = np.zeros((29, ), dtype=np.float64)

    # --------------------------------------------------------
    def quaternion_to_euler(self, q):
        """
        Convert a quaternion to Euler angles (roll, pitch, yaw).
        """
        w, x, y, z = q

        # roll (X-axis rotation)
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)

        # pitch (Y-axis rotation)
        t2 = 2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)  # 防止数值漂移导致 |t2|>1
        pitch = np.arcsin(t2)

        # yaw (Z-axis rotation)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)

        return np.array([roll, pitch, yaw], dtype=np.float32)

    # --------------------------------------------------------
    

    def _send_robot_state(self, step: int):
        # ----- IMU -----
        q_world = self.data.sensordata[:4]  # quaternion
        # rpy       = mujoco.mju_mat2Euler(mujoco.mju_quat2Mat(q_world))
        rpy = self.quaternion_to_euler(q_world)
        # rpy = Rotation.from_quat(q_world).as_euler('ZYX', degrees=False)

        # linvel    = self.data.qvel[3:6]             # world frame
        body_acc = self.data.sensordata[4:7]
        angvel_b = self.data.sensordata[7:10]  # body frame

 
        # ----- 关节 -----
        q = self.data.qpos[ : self.dof_num]
        dq = self.data.qvel[ : self.dof_num]
        tau = self.input_tq.flatten()
        # print(f"[IMU] tau: {tau}")

        # --- 调试打印 ---

        # print(f"[IMU] RPY: {rpy.flatten()}")
        # print(f"[IMU] Omega: {angvel_b.flatten()}")
        # print(f"[IMU] Acc_body: {body_acc.flatten()}")

        # 打包并发送
        # payload = np.concatenate((
        #     np.array([self.timestamp], dtype=np.float64),
        #     np.asarray(rpy, dtype=np.float32),
        #     np.asarray(body_acc, dtype=np.float32),
        #     np.asarray(angvel_b, dtype=np.float32),
        #     q.astype(np.float32),
        #     dq.astype(np.float32),
        #     tau.astype(np.float32)
        # ))
        
        payload = np.concatenate([
            [self.timestamp],
            rpy.flatten(),
            body_acc.flatten(),
            angvel_b.flatten(),
            q,
            dq,
            tau,
        ]).astype(np.float32)
        # print(self.timestamp)
        self.pyload_old = payload

        fmt = "1d" + f"{len(payload) - 1}f"
        try:
            # self.send_sock.sendto(struct.pack(fmt, *payload), self.ctrl_addr)
            self.send_sock.sendto(
                struct.pack(f'{len(payload)}f', *payload),
                self.ctrl_addr
            )
        except socket.error as ex:
            print(f"[UDP send] {ex}")


# ------------------ main ------------------
if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    sim = MuJoCoSimulation()
    sim.start()
