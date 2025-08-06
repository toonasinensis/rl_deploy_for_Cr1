import pybullet as p
import numpy as np
import pybullet_data as pd
import os
import socket
import struct
import threading
import time
from typing import Dict, List, Tuple

# Configuration constants
URDF_PATHS: Dict[str, str] = {
    "k1w": "/K1W/urdf/K1W.urdf",
    "m20": "/CA9B/urdf/CA9B.urdf",
    "CR1LEG": "/CR01A-half/CR01A.urdf",
    "CR1PRO": "/CR01B-pro/B-20250522.urdf",
    "CR1STANDARD": "/CR01B-standard/B-20250522.urdf",
}

INIT_JOINT_POSITIONS: Dict[str, List[float]] = {
    "k1w": [0, -1.35453, 2.54948, 0] * 4,
    "m20": [0, -1.35453, 2.54948, 0,
            0, -1.35453, 2.54948, 0,
            0, 1.35453, -2.54948, 0,
            0, 1.35453, -2.54948, 0],
    "CR1LEG": [0, ] * 12,
    "CR1PRO": [0, ] * 29,
    "CR1STANDARD": [0, ] * 21,
}

INIT_Z_POS: Dict[str, float] = {
    "k1w": 0.4,
    "m20": 0.4,
    "CR1LEG": 1.0,
    "CR1PRO": 0.3,
    "CR1STANDARD": 0.3,
}

virtual_joint_num = {
    "k1w": 0,
    "m20": 0,
    "CR1LEG": 0,
    "CR1PRO": 0,
    "CR1STANDARD": 0,
}


class PyBulletSimulation:
    """PyBullet simulation environment for legged robot locomotion.

    Attributes:
        robot (int): PyBullet body ID of the loaded robot
        jointIdxList (List[int]): List of controllable joint indices
        inputTorque (np.ndarray): Array of joint torques to be applied
    """

    def __init__(self, robot_name: str, urdf_path: str,
                 local_port: int = 20001, ctrl_ip: str = "127.0.0.1",
                 ctrl_port: int = 30010) -> None:
        """Initialize simulation environment.

        Args:
            robot_name: Name of the robot model
            urdf_path: Path to robot URDF file
            local_port: Local UDP port for receiving control commands
            ctrl_ip: IP address for sending robot data
            ctrl_port: Port for sending robot data
        """
        self._setup_communication(local_port, ctrl_ip, ctrl_port)
        self._init_pybullet_env()
        self._load_robot(robot_name, urdf_path)
        self._create_terrain()
        self._init_simulation_vars()
        self.virtual_joint_num = virtual_joint_num[robot_name]
        if virtual_joint_num[robot_name] > 0:
            self.virtual_joint = np.zeros(virtual_joint_num[robot_name])
        else:
            self.virtual_joint = None
        self.robot_name = robot_name

    def _setup_communication(self, local_port: int, ctrl_ip: str,
                             ctrl_port: int) -> None:
        """Initialize network communication settings."""
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server.settimeout(5)
        self.ctrl_addr = (ctrl_ip, ctrl_port)
        self.local_port = local_port

    def _init_pybullet_env(self) -> None:
        """Initialize PyBullet graphical environment and physics settings."""
        p.connect(p.GUI)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 0)
        p.setGravity(0, 0, -9.81)
        p.setAdditionalSearchPath(pd.getDataPath())
        self.terrain = p.loadURDF("plane.urdf")

    def _load_robot(self, robot_name: str, urdf_path: str) -> None:
        """Load robot model into simulation.

        Args:
            robot_name: Name of the robot configuration to use
            urdf_path: Path to URDF file relative to project root
        """
        z_init = INIT_Z_POS[robot_name]
        current_dir = os.path.dirname(os.path.abspath(__file__))
        print(current_dir)
        full_urdf_path = f"{current_dir}/urdf_model/{urdf_path}"

        self.robot = p.loadURDF(
            full_urdf_path,
            useMaximalCoordinates=False,
            basePosition=[0.0, 0, z_init],
            baseOrientation=p.getQuaternionFromEuler([0., -1.57, 0.]),
            flags=p.URDF_USE_INERTIA_FROM_FILE |
                  p.URDF_USE_SELF_COLLISION |
                  p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS,
            useFixedBase=False
        )
        num_joints = p.getNumJoints(self.robot)
        for i in range(num_joints):
            info = p.getJointInfo(self.robot, i)
            print(i, info[1].decode("utf-8"))

        self.jointIdxList = []
        for j in range(p.getNumJoints(self.robot)):
            joint_info = p.getJointInfo(self.robot, j)
            if joint_info[2] == p.JOINT_REVOLUTE:
                self.jointIdxList.append(j)
                p.resetJointState(
                    self.robot, j,
                    INIT_JOINT_POSITIONS[robot_name][len(self.jointIdxList) - 1]
                )
                p.setJointMotorControl2(self.robot, j, p.VELOCITY_CONTROL, force=0)

        self.dof_num = len(self.jointIdxList)
        print(f"Controllable joints: {self.jointIdxList}")

    def _create_terrain(self) -> None:
        """Generate complex terrain with stairs and slopes."""
        self._create_stairs()
        self._create_slopes()
        self._create_step(0, 2., 0.45, 2, 2, 0.9, friction=2.0)

    def _create_stairs(self) -> None:
        """Generate staircase with ascending and descending sections."""
        # Stair parameters
        stair_params = {
            'step_length': 0.3,
            'step_width': 2.5,
            'step_height': 0.15,
            'num_steps_up': 6,
            'platform_length': 1.0,
            'num_steps_down': 6,
            'start_x': 1.5,
            'friction': 1.0
        }

        # Create ascending stairs
        for i in range(stair_params['num_steps_up']):
            x = i * stair_params['step_length'] + stair_params['step_length'] / 2 + stair_params['start_x']
            z = i * stair_params['step_height'] + stair_params['step_height'] / 2
            self._create_step(x, 0, z, stair_params['step_length'],
                              stair_params['step_width'], stair_params['step_height'],
                              stair_params['friction'])

        # Create platform
        platform_x = (stair_params['num_steps_up'] * stair_params['step_length']
                      + stair_params['start_x'] + stair_params['platform_length'] / 2)
        platform_z = stair_params['num_steps_up'] * stair_params['step_height']
        self._create_step(platform_x, 0, platform_z - stair_params['step_height'] / 2,
                          stair_params['platform_length'], stair_params['step_width'],
                          stair_params['step_height'],
                          stair_params['friction'])

        # Create descending stairs
        for i in range(stair_params['num_steps_down']):
            x = (platform_x + stair_params['platform_length'] / 2
                 + i * stair_params['step_length'] + stair_params['step_length'] / 2)
            z = platform_z - (i + 1) * stair_params['step_height'] + stair_params['step_height'] / 2
            self._create_step(x, 0, z, stair_params['step_length'],
                              stair_params['step_width'], stair_params['step_height'],
                              stair_params['friction'])

    def _create_slopes(self) -> None:
        """Generate sloped terrain with inclines and declines."""
        # Slope parameters
        slope_params = {
            'slope_length': 3.0,
            'slope_height': 1.0,
            'thickness': 0.01,
            'width': 3.0,
            'platform_length': 2.5,
            'start_x': -1.5,
            'friction': 2.0
        }

        # Calculate common parameters
        theta = np.arctan(slope_params['slope_height'] / slope_params['slope_length'])

        # Create upward slope
        up_slope_x = slope_params['start_x'] - slope_params['slope_length'] / 2
        self._create_slope_segment(
            up_slope_x, theta,
            base_z=slope_params['thickness'] / 2 + slope_params['slope_height'] / 2,
            friction=slope_params['friction']
        )

        # Create platform
        platform_x = slope_params['start_x'] - slope_params['slope_length'] - slope_params['platform_length'] / 2
        self._create_step(
            platform_x, 0, slope_params['slope_height'] - slope_params['thickness'] / 2,
            slope_params['platform_length'], slope_params['width'], slope_params['thickness'],
            slope_params['friction']
        )

        # Create downward slope
        down_slope_x = platform_x - slope_params['platform_length'] / 2 - slope_params['slope_length'] / 2
        self._create_slope_segment(
            down_slope_x, -theta,
            base_z=slope_params['slope_height'] - slope_params['thickness'] / 2 - slope_params['slope_height'] / 2,
            friction=slope_params['friction']
        )

    def _create_step(self, x: float, y: float, z: float,
                     length: float, width: float, height: float, friction=1.0) -> None:
        """Create a single step/box obstacle."""
        step = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[length / 2, width / 2, height / 2]
        )
        p.createMultiBody(0, step, basePosition=[x, y, z])
        p.changeDynamics(step, -1, lateralFriction=friction)

    def _create_slope_segment(self, x: float, angle: float, base_z: float, friction=1.0) -> None:
        """Create a sloped terrain segment."""
        actual_length = np.sqrt(3.0 ** 2 + 1.0 ** 2)
        slope = p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=[actual_length / 2, 1.5, 0.005]
        )
        orientation = p.getQuaternionFromEuler([0, angle, 0])
        slope = p.createMultiBody(
            0, slope,
            basePosition=[x, 0, base_z],
            baseOrientation=orientation
        )
        p.changeDynamics(slope, -1, lateralFriction=friction)

    def _init_simulation_vars(self) -> None:
        """Initialize simulation state variables."""
        self.inputTorque = np.zeros((self.dof_num, 1))
        self.lastBaseVelWorld = np.zeros((3, 1))
        self.lastRotMat = np.eye(3)

        # Initialize command buffers
        self.kpCmd = np.zeros((self.dof_num, 1))
        self.kdCmd = np.zeros((self.dof_num, 1))
        self.jointPosCmd = np.zeros((self.dof_num, 1))
        self.jointVelCmd = np.zeros((self.dof_num, 1))
        self.tauCmd = np.zeros((self.dof_num, 1))

    def start_simulation(self) -> None:
        """Main simulation loop with physics updates and rendering."""
        p.setTimeStep(0.001)
        simulation_thread = threading.Thread(target=self._receive_joint_cmds)
        simulation_thread.daemon = True
        simulation_thread.start()
        run_cnt = 0

        while True:
            start_time = time.time()
            run_cnt += 1
            self.timestamp = run_cnt * 0.001
            self._simulation_step()
            self._enforce_real_time(start_time)

    def _simulation_step(self) -> None:
        """Perform single simulation step."""
        self.get_joint_states()
        self.get_imu_data()
        self.send_robot_data()
        self._update_joint_torques()
        self._update_camera_view()
        p.stepSimulation()

    def _update_joint_torques(self) -> None:
        """Calculate and apply joint torques using PD control."""
        self.inputTorque = (np.multiply(self.kpCmd, (self.jointPosCmd - self.jointPos))
                            + np.multiply(self.kdCmd, (self.jointVelCmd - self.jointVel))
                            + self.tauCmd)

        p.setJointMotorControlArray(
            self.robot, self.jointIdxList,
            controlMode=p.TORQUE_CONTROL,
            forces=self.inputTorque.reshape(self.dof_num).tolist()
        )

    def _update_camera_view(self) -> None:
        """Update debug camera to follow robot position."""
        base_pos = p.getBasePositionAndOrientation(self.robot)[0]
        focus_pos = [base_pos[0], base_pos[1], base_pos[2] + 0.25]
        camInfo = p.getDebugVisualizerCamera()
        curTargetPos = camInfo[11]
        distance = camInfo[10]  # 相机的距离，没什么用
        yaw = camInfo[8]
        pitch = camInfo[9]
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=yaw,
            cameraPitch=pitch,
            cameraTargetPosition=focus_pos,
        )

    def _enforce_real_time(self, start_time: float) -> None:
        """Maintain real-time simulation speed."""
        elapsed = time.time() - start_time
        if elapsed < 0.001:
            time.sleep(0.001 - elapsed)

    def get_imu_data(self) -> None:
        """Calculate and update IMU sensor data (orientation, angular velocity, linear acceleration)."""
        _, base_quat = p.getBasePositionAndOrientation(self.robot)
        base_vel_world, base_omega_world = p.getBaseVelocity(self.robot)
        rot_mat = np.array(p.getMatrixFromQuaternion(base_quat)).reshape(3, 3)

        # Calculate orientation
        self.base_rpy = np.array(p.getEulerFromQuaternion(base_quat)).reshape(3, 1)
        # self.base_quat = base_quat.reshape(4, 1)
        # Calculate angular velocity in body frame
        self.base_omega = rot_mat.T @ np.array(base_omega_world).reshape(3, 1)

        # Calculate linear acceleration in body frame
        base_vel_world = np.array(base_vel_world).reshape(3, 1)
        base_acc_world = (base_vel_world - self.lastBaseVelWorld) / 0.001
        self.lastBaseVelWorld = base_vel_world
        base_acc_world[2] += 9.81  # Add gravity compensation
        self.base_acc = rot_mat.T @ base_acc_world

    def get_joint_states(self) -> None:
        """Retrieve current joint positions, velocities, and torques."""
        joint_states = p.getJointStates(self.robot, self.jointIdxList)
        self.jointPos = np.array([s[0] for s in joint_states]).reshape(-1, 1)
        self.jointVel = np.array([s[1] for s in joint_states]).reshape(-1, 1)
        # print("self.jointPos", self.jointPos.shape)
        # self.inputTorque = np.array([s[3] for s in joint_states]).reshape(-1, 1)

    def send_robot_data(self) -> None:
        """Package and send robot sensor data via UDP."""
        if self.virtual_joint is None:
            data_packet = np.concatenate([
                [self.timestamp],
                self.base_rpy.flatten(),
                # self.base_quat.flatten(),
                self.base_acc.flatten(),
                self.base_omega.flatten(),
                self.jointPos.flatten(),
                self.jointVel.flatten(),
                self.inputTorque.flatten()
            ])
        else:
            data_packet = np.concatenate([
                [self.timestamp],
                self.base_rpy.flatten(),
                self.base_acc.flatten(),
                self.base_omega.flatten(),
                np.concatenate([self.jointPos.flatten(), self.virtual_joint]),
                np.concatenate([self.jointVel.flatten(), self.virtual_joint]),
                np.concatenate([self.inputTorque.flatten(), self.virtual_joint]),
            ])
            # print('base_rpy:', self.base_rpy.flatten())
            # print('base_acc:', self.base_acc.flatten())
            # print('base_omega:', self.base_omega.flatten())
            # print('jointPos:', self.jointPos.flatten())
            # print('inputTorque:', self.inputTorque.flatten())
            # np.set_printoptions(precision=4, suppress=True)
            # print(len(data_packet))

        try:
            self.server.sendto(
                struct.pack(f'{len(data_packet)}f', *data_packet),
                self.ctrl_addr
            )
        except socket.error as e:
            print(f"Network error: {str(e)}")

    def _receive_joint_cmds(self) -> None:
        """UDP listener thread for receiving joint commands."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('127.0.0.1', self.local_port))

        fmt = f'{self.dof_num + self.virtual_joint_num}f'
        chunk_size = struct.calcsize(fmt)

        while True:
            try:
                data = sock.recv(5 * chunk_size)
                params = [struct.unpack_from(fmt, data, offset=i * chunk_size)
                          for i in range(5)]
                if self.robot_name == 'CR1PRO':
                    
                    self.kpCmd = np.array(params[0]).reshape(-1, 1)#[:-2]
                    # print(self.kpCmd.transpose())
                    # self.jointPosCmd = np.array(params[1]).reshape(-1, 1)#[:-2]
                    # self.kdCmd = np.array(params[2]).reshape(-1, 1)#[:-2]
                    # self.jointVelCmd = np.array(params[3]).reshape(-1, 1)#[:-2]
                    # self.tauCmd = np.array(params[4]).reshape(-1, 1)#[:-2]
                else:
                    self.kpCmd = np.array(params[0]).reshape(-1, 1)
                    self.jointPosCmd = np.array(params[1]).reshape(-1, 1)
                    self.kdCmd = np.array(params[2]).reshape(-1, 1)
                    self.jointVelCmd = np.array(params[3]).reshape(-1, 1)
                    self.tauCmd = np.array(params[4]).reshape(-1, 1)
            except (socket.timeout, struct.error) as e:
                print(f"Command reception error: {str(e)}")


if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True)
    robot_name = "CR1PRO"
    sim = PyBulletSimulation(robot_name, URDF_PATHS[robot_name])
    sim.start_simulation()
