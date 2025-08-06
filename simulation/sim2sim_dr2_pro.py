import numpy as np
import mujoco
import mujoco.viewer

from tqdm import tqdm
from collections import deque
from scipy.spatial.transform import Rotation as R
from mimic_real.motion_loader import MotionSampler, MotionDataCfg
 
# from pynput import keyboard
import random
import math
import torch
import time

import re
from mimic_real import *
from mimic_real.data import MOTION_DATA_DIR
np.set_printoptions(precision=3, suppress=True)
import joblib
import json
import os

from  pathlib import Path

class env:
    num_single_obs = 98
    num_his = 5
    num_his_obs = num_single_obs * num_his
    num_actions = 23
    num_all_joints = 29

    joint_names_lab =  ['left_hip_y_joint', 'right_hip_y_joint', 'waist_z_joint', 'left_hip_x_joint', 'right_hip_x_joint', 'waist_x_joint', 'left_hip_z_joint', 'right_hip_z_joint', 'waist_y_joint', 'left_knee_joint', 'right_knee_joint', 'left_shoulder_y_joint', 'right_shoulder_y_joint', 'left_ankle_y_joint', 'right_ankle_y_joint', 'left_shoulder_x_joint', 'right_shoulder_x_joint', 'left_ankle_x_joint', 'right_ankle_x_joint', 'left_shoulder_z_joint', 'right_shoulder_z_joint', 'left_elbow_joint', 'right_elbow_joint']
    joint_no_rl = ["left_wrist_z_joint", "left_wrist_y_joint", "left_wrist_x_joint",
        "right_wrist_z_joint", "right_wrist_y_joint", "right_wrist_x_joint"]
    joint_rl_and_no_rl_lab = joint_names_lab + joint_no_rl
    link_names_lab = ['base_link', 'left_hip_y_link', 'right_hip_y_link', 'waist_z_link', 'left_hip_x_link', 'right_hip_x_link', 'waist_x_link', 'left_hip_z_link', 'right_hip_z_link', 'body', 'left_knee_link', 'right_knee_link', 'left_shoulder_y_link', 'mass_point_1', 'mass_point_2', 'mass_point_3', 'mass_point_4', 'mass_point_5', 'mass_point_6', 'mass_point_7', 'mass_point_8', 'right_shoulder_y_link', 'left_ankle_y_link', 'right_ankle_y_link', 'left_shoulder_x_link', 'right_shoulder_x_link', 'left_ankle_x_link', 'right_ankle_x_link', 'left_shoulder_z_link', 'right_shoulder_z_link', 'left_elbow_link', 'right_elbow_link', 'left_wrist_z_link', 'right_wrist_z_link', 'left_wrist_y_link', 'right_wrist_y_link', 'left_wrist_x_link', 'right_wrist_x_link']
    
    default_joint_pos={
            ".*_hip_y_joint": -0.20,
            ".*_knee_joint": 0.42,
            ".*_ankle_y_joint": -0.23,
            ".*_elbow_joint": 0.87,
            "left_shoulder_x_joint": 0.18,
            "left_shoulder_y_joint": 0.35,
            "right_shoulder_x_joint": -0.18,
            "right_shoulder_y_joint": 0.35,
        }
    device = "cpu"
    joint_names_mujoco = ['waist_z_joint', 'waist_x_joint', 'waist_y_joint', 'left_shoulder_y_joint', 'left_shoulder_x_joint', 'left_shoulder_z_joint', 'left_elbow_joint', 'left_wrist_z_joint', 'left_wrist_y_joint', 'left_wrist_x_joint', 'right_shoulder_y_joint', 'right_shoulder_x_joint', 'right_shoulder_z_joint', 'right_elbow_joint', 'right_wrist_z_joint', 'right_wrist_y_joint', 'right_wrist_x_joint', 'left_hip_y_joint', 'left_hip_x_joint', 'left_hip_z_joint', 'left_knee_joint', 'left_ankle_y_joint', 'left_ankle_x_joint', 'right_hip_y_joint', 'right_hip_x_joint', 'right_hip_z_joint', 'right_knee_joint', 'right_ankle_y_joint', 'right_ankle_x_joint']
    link_names_mujoco = ['base_link', 'waist_z_link', 'waist_x_link', 'body', 'left_shoulder_y_link', 'left_shoulder_x_link', 'left_shoulder_z_link', 'left_elbow_link', 'left_wrist_z_link', 'left_wrist_y_link', 'left_wrist_x_link', 'right_shoulder_y_link', 'right_shoulder_x_link', 'right_shoulder_z_link', 'right_elbow_link', 'right_wrist_z_link', 'right_wrist_y_link', 'right_wrist_x_link', 'left_hip_y_link', 'left_hip_x_link', 'left_hip_z_link', 'left_knee_link', 'left_ankle_y_link', 'left_ankle_x_link', 'right_hip_y_link', 'right_hip_x_link', 'right_hip_z_link', 'right_knee_link', 'right_ankle_y_link', 'right_ankle_x_link']
    # 初始化默认位置数组
    default_positions = np.zeros(len(joint_names_lab))

    # 为每个关节查找匹配的默认位置
    for i, joint_name in enumerate(joint_names_lab):
        matched = False
        for pattern, default_value in default_joint_pos.items():
            if re.fullmatch(pattern, joint_name):
                default_positions[i] = default_value
                matched = True
                break
        
        # 如果没有匹配到任何模式，保持为0（或者你可以设置一个默认值）
        if not matched:
            default_positions[i] = 0.0  # 或者其他默认值

    print("生成的默认关节位置数组:")
    print(default_positions)


class Sim2simCfg:
    class sim_config:
        mujoco_model_path = MIMIC_ASSETS_DIR + 'DR2/DR2.xml'
        model_path =  "/home/tian/Desktop/learn/imation/mimic/logs/DR2_mimic/test/exported/policy.pt"
        motion_path = MOTION_DATA_DIR + "/g1_single/dance.json"
        sim_duration = 60.0

        rl_dt = 0.02
        dt = 0.001
        decimation = int(rl_dt / dt)
        
    class robot_config:        
        pdtv_dict = {
            # legs
            "left_hip_z_joint": (100, 2.5, 107.0),
            "right_hip_z_joint": (100, 2.5, 107.0),

            "left_hip_x_joint": (150, 3.75, 107.0),
            "right_hip_x_joint": (150, 3.75, 107.0),

            "left_hip_y_joint": (150, 3.75, 413.0),
            "right_hip_y_joint": (150, 3.75, 413.0),

            "left_knee_joint": (150, 3.75, 413.0),
            "right_knee_joint": (150, 3.75, 413.0),

            # feet
            "left_ankle_y_joint": (100.0, 2.5, 107.0),
            "right_ankle_y_joint": (100.0, 2.5, 107.0),

            "left_ankle_x_joint": (30.0, 1., 31.0),
            "right_ankle_x_joint": (30.0, 1., 31.0),

            # waist
            "waist_z_joint": (120, 3, 107.0),
            "waist_y_joint": (120, 3, 413.0),
            "waist_x_joint": (120, 3, 107.0),

            # shoulders
            "left_shoulder_y_joint": (80.0, 2.0, 107.0),
            "right_shoulder_y_joint": (80.0, 2.0, 107.0),

            "left_shoulder_x_joint": (80.0, 2.0, 107.0),
            "right_shoulder_x_joint": (80.0, 2.0, 107.0),

            # arms
            "left_shoulder_z_joint": (80.0, 2.0, 107.0),
            "right_shoulder_z_joint": (80.0, 2.0, 107.0),

            "left_elbow_joint": (60.0, 1.50, 107.0),
            "right_elbow_joint": (60.0, 1.50, 107.0),

            # wrist
            "left_wrist_z_joint": (60.0, 3.0, 31.0),
            "right_wrist_z_joint": (60.0, 3.0, 31.0),

            "left_wrist_y_joint": (60.0, 3.0, 31.0),
            "right_wrist_y_joint": (60.0, 3.0, 31.0),

            "left_wrist_x_joint": (60.0, 3.0, 31.0),
            "right_wrist_x_joint": (60.0, 3.0, 31.0),
        }


        kps = np.zeros(env.num_all_joints)
        kds = np.zeros(env.num_all_joints)
        tau_limit = np.zeros(env.num_all_joints)
        for name in env.joint_names_mujoco:
            kps[env.joint_names_mujoco.index(name)] = pdtv_dict[name][0]
            kds[env.joint_names_mujoco.index(name)] = pdtv_dict[name][1]
            tau_limit[env.joint_names_mujoco.index(name)] = pdtv_dict[name][2]
            
        use_filter = False


motion_data = MotionDataCfg()
motion_data.motion_file_path = MOTION_DATA_DIR + "/dr2/mj/"        
# assert(motion_data.motion_file_path == '/home/tian/Desktop/learn/imation/mimic/mimic_real/data/dr2/B1 - stand to walk_poses.json')
# print("gfhhfhoeovfidh")
motion_data.privi_capture_points_link_names = [
    'left_hip_z_link', "left_knee_link", "left_ankle_x_link", 
    'right_hip_z_link', 'right_knee_link', 'right_ankle_x_link', \
    "left_shoulder_x_link", "left_elbow_link", 
    "right_shoulder_x_link", "right_elbow_link", 
    "left_wrist_x_link",
    "right_wrist_x_link",
    "base_link" ,"body"]
motion_data.urdf_file: str = "mimic_real/assets/DR2_fixed_wrist/DR2.urdf" 
motion_data.target_link_names: list =  [
            'left_hip_z_link', "left_knee_link", "left_ankle_x_link", 
            'right_hip_z_link', 'right_knee_link', 'right_ankle_x_link', \
            "left_shoulder_x_link", "left_elbow_link", 
            "right_shoulder_x_link", "right_elbow_link", 
            "left_wrist_x_link",
            "right_wrist_x_link",
            "base_link" ,"body",
            "mass_point_1",
            "mass_point_2",
            "mass_point_3",
            "mass_point_4",
            "mass_point_5",
            "mass_point_6",
            "mass_point_7",
            "mass_point_8",
            ]


def quat_rotate_inverse(q, v):
    q_w = q[-1]
    q_vec = q[:3]
    a = v * (2.0 * q_w**2 - 1.0)
    b = np.cross(q_vec, v) * q_w * 2.0
    c = q_vec * np.dot(q_vec, v) * 2.0
    return a - b + c

def quat_rotate_inverse_batch(q, v):
    q_w = q[-1]
    q_vec = q[:3]
    a = v * (2.0 * q_w**2 - 1.0)
    # print("11111")
    b = np.cross(q_vec, v) * q_w * 2.0
    # import ipdb;ipdb.set_trace();

    c =  np.dot(v, q_vec) [:, None] *  q_vec[None, :] *2.0
    return a - b + c

def quat_apply(quat: np.array, vec: np.array) -> np.array:
    # assert quat.shape == (1, 4) and vec.shape == (1, 3)
    xyz = quat[:3]
    w = quat[-1]
    t = np.cross(xyz, vec) * 2
    return (vec + w * t + np.cross(xyz, t)) # xyz.cross(t, dim=-1)


def get_obs(data):
    """Extracts an observation from the mujoco data structure"""
    qpos = data.qpos.astype(np.double)
    dq = data.qvel.astype(np.double)
    quat = data.sensor("orientation").data[[1, 2, 3, 0]].astype(np.double)
    r = R.from_quat(quat)
    v = r.apply(data.qvel[:3], inverse=True).astype(np.double)  # In the base frame
    omega = data.sensor("angular-velocity").data.astype(np.double)
    gvec = r.apply(np.array([0.0, 0.0, -1.0]), inverse=True).astype(np.double)
    return (qpos, dq, quat, v, omega, gvec)

def get_body_pos(body_name_list:str,model,data):
    body_pos_list = []
    for name in body_name_list:
        body_id = mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,name)
        body_pos= data.xpos[body_id]
        body_pos_list.append(body_pos)
    return np.array(body_pos_list)

def low_pass_action_filter(actions, last_actions):
  alpha = 0.2
  actons_filtered = last_actions * alpha + actions * (1 - alpha)
  return actons_filtered


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd 

def get_trans_index(joint_names_lab, joint_names_mujoco):
    assert len(joint_names_lab) == len(joint_names_mujoco)
    joint_num = len(joint_names_mujoco)
    mjc2lab_list = []
    lab2mjc_list = []
    for i in range(joint_num):
        mjc2lab_list.append(joint_names_lab.index(joint_names_mujoco[i]))
        lab2mjc_list.append(joint_names_mujoco.index(joint_names_lab[i]))
    return mjc2lab_list, lab2mjc_list

def view_motion(data,motion_loader_mj,ts_phase):
    data.qpos[-env.num_actions :] = motion_loader_mj.get_dof_pos_batch(phase=ts_phase)[0].cpu().numpy()
    data.qpos[0:3] = motion_loader_mj.get_root_trans_batch(phase=ts_phase)[0].cpu().numpy()
    data.qpos[3:7] = motion_loader_mj.get_root_rot_batch(phase=ts_phase)[0].cpu().numpy()
    data.qvel[0:3] = motion_loader_mj.get_root_vel_batch(phase=ts_phase)[0].cpu().numpy()
    data.qvel[3:6] = motion_loader_mj.get_root_omega_batch(phase=ts_phase)[0].cpu().numpy()
    data.qvel[-env.num_actions:] = motion_loader_mj.get_dof_vel_batch(phase=ts_phase)[0].cpu().numpy()


import onnxruntime as ort
import numpy as np

class data_save:   #_for_deploy ， 注意这里保存的是50HZ
    des_joint_pos = []

def run_mujoco():
    # save_data = data_save_for_deploy()
    session = ort.InferenceSession("/home/tian/Desktop/learn/imation/mimic/logs/DR2_mimic/test/exported/tmppolicy.onnx", providers=['CPUExecutionProvider'])

    # 获取输入名（通常只有一个）
    input_name = session.get_inputs()[0].name

    motion_loader_lab = MotionSampler(cfg=motion_data,simulator_joint_names=env.joint_names_lab,\
                                      simulator_body_link_names=env.link_names_lab,num_envs=1,device=env.device)
    
    # motion_loader_mj = MotionSampler(cfg=motion_data,simulator_joint_names=env.joint_names_mujoco,\
    #                                   simulator_body_link_names=env.link_names_mujoco,num_envs=1,device=env.device)
    
    # motion_loader_lab = MotionLoader(Sim2simCfg.sim_config.motion_path, env.joint_names_lab, device=env.device, add_static_frame=False)

    st1 = time.time()
    model = mujoco.MjModel.from_xml_path(Sim2simCfg.sim_config.mujoco_model_path)
    st2 = time.time()
    print("加载模型用时",st2 - st1)
    model.opt.timestep = Sim2simCfg.sim_config.dt
    mjc2lab_list, lab2mjc_list = get_trans_index(env.joint_rl_and_no_rl_lab, env.joint_names_mujoco)

    data = mujoco.MjData(model)
    joint_lab = motion_loader_lab.get_dof_pos_batch(phase=torch.Tensor([0]))[0].cpu().numpy()
    joint_lab_all = np.zeros((29,), dtype=np.float32)
    joint_lab_all[:23] = joint_lab
    joint_mj = np.zeros((29,))
    for i in range(env.num_all_joints):
        joint_mj[i] = joint_lab_all[mjc2lab_list[i]]
    data.qpos[-env.num_all_joints:] = joint_mj
    data.qpos[0:3] = motion_loader_lab.get_root_trans_batch(phase=torch.Tensor([0]))[0].cpu().numpy()
    data.qpos[3:7] = motion_loader_lab.get_root_rot_batch(phase=torch.Tensor([0]))[0].cpu().numpy()
    print("joint_mj",joint_mj)
    print("data.qpos[0:3]",data.qpos[0:3])
    print("data.qpos[3:7]",data.qpos[3:7])
    mujoco.mj_step(model, data)

    last_action_lab = np.zeros((1, env.num_actions), dtype=np.double)
    hist_obs = np.zeros([1, env.num_his_obs])

    count_lowlevel = 0

    model_path = Sim2simCfg.sim_config.model_path
    policy_jit = torch.jit.load(model_path)

   
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Close the viewer automatically after simulation_duration wall-seconds.
        start = time.time()
        while viewer.is_running() and time.time() - start < 1000:

            step_start = time.time()
            # Obtain an observation
            q_mj, dq_mj, quat, v, omega, gvec = get_obs(data)
            # view_center = q[0:3]
            
            # import ipdb; ipdb.set_trace();
            q = q_mj[-env.num_all_joints :]
            dq = dq_mj[-env.num_all_joints :]

            q_lab_all = np.zeros_like(q)
            dq_lab_all = np.zeros_like(dq)
            for i in range(env.num_actions):
                q_lab_all[i] = q[lab2mjc_list[i]]
                dq_lab_all[i] = dq[lab2mjc_list[i]]    

                q_lab = q_lab_all[0:env.num_actions]
                dq_lab = dq_lab_all[:env.num_actions]
            phase = (count_lowlevel * Sim2simCfg.sim_config.dt) / motion_loader_lab.get_record_time()
            # print("phase",phase)
            if phase >= 1:
                current_dir = Path(__file__).resolve().parent
                print(current_dir)
                # print()
                # import ipdb;ipdb.set_trace();
                data_out = {
                     "des_joint_pos": [arr.tolist() for arr in data_save.des_joint_pos],
                }
                file_path = current_dir / "output" / "data_output.json"
                print("保存必要的部署数据！,save data for deploy")
                with open(file_path, 'w') as f:
                    json.dump(data_out, f, indent=2)
                
            phase %= 1
            # 1000hz -> 50hz        
            if count_lowlevel % Sim2simCfg.sim_config.decimation == 0:
                # phase = 0
                ts_phase = torch.Tensor([phase])
                obs = np.zeros([1, env.num_single_obs])
                _q = quat
                _v = np.array([0.0, 0.0, -1.0])
                projected_gravity = quat_rotate_inverse(_q, _v)
                default_q = env.default_positions
                q_des = motion_loader_lab.get_dof_pos_batch(phase=ts_phase)[0].cpu().numpy()
                
                body_pos_lab = get_body_pos(motion_data.target_link_names,model=model,data=data)
                root_xyz = body_pos_lab[motion_data.target_link_names.index("base_link"),:].copy()
                # des_root_z = body_pos_lab[motion_data.target_link_names.index("base_link"),:].copy()


                #save data for deploy：
                data_save.des_joint_pos.append(q_des)
                # print(data_save.des_joint_pos)
                # root_xyz[2] = 0
                                        
                obs = np.concatenate((
                    omega.flatten()[None,:],#0:3
                    projected_gravity.flatten()[None,:],#3:6
                    q_lab.flatten()[None,:] - default_q.flatten()[None,:],#6:35
                    dq_lab.flatten()[None,:],#35:74
                    # q_err[None, :],
                    last_action_lab.flatten()[None,:],#153:172
                    q_des.flatten()[None,:], #74:103
                ), axis = -1)
                # print("omega",omega)
                # print("projected_gravity",projected_gravity)
                # print("q_lab",q_lab.flatten()[None,:] - default_q.flatten()[None,:])
                # print("dq_lab",dq_lab)
                # print("last_action_lab",last_action_lab)
                # print("q_des",q_des)
                # import ipdb;ipdb.set_trace();
                assert(env.num_single_obs == obs.shape[1])
                hist_obs = np.concatenate((hist_obs[:, env.num_single_obs:], obs[:, :env.num_single_obs]), axis=-1).astype(np.float32)
                # action_lab = policy_jit(torch.from_numpy(hist_obs.astype(np.float32))).detach().numpy() # TODO

                action_lab = session.run(None, {input_name: hist_obs.astype(np.float32)})[0]
                # action_lab = np.zeros((1, env.num_actions), dtype=np.double)
                last_action_lab[:] = action_lab[:]
        
                # action_lab = np.clip(action_lab, -0.0, 0.0)
                # print("action_lab", action_lab)
                action_all_lab = np.zeros((1,29),dtype=np.float32)
                action_all_lab[:, :23] = action_lab
                default_q_all = np.zeros((1,29),dtype=np.float32)
                default_q_all[:,:23] = default_q
                target_q_lab = action_all_lab * 0.25 + default_q_all
                # import ipdb;ipdb.set_trace();
                target_q_mjc = np.zeros_like(target_q_lab)
                for i in range(env.num_all_joints):
                    target_q_mjc[0][i] = target_q_lab[0][mjc2lab_list[i]]
                target_q_mujoco = target_q_mjc[0]  # (29, ) 展平
                
                # dof_pos_ref = motion_loader_mj.get_dof_pos_batch(phase=torch.Tensor([phase]))[0].cpu().numpy()
                # for i in range(env.num_actions):
                #     target_q_mujoco[i] = target_q_mujoco[i] # + target_q_delta_mjc[0][i] 

           
            target_dq = np.zeros(env.num_all_joints, dtype=np.double)
            tau  = pd_control(
                target_q_mujoco, q, Sim2simCfg.robot_config.kps, target_dq, dq, Sim2simCfg.robot_config.kds
            )
            # tau = np.clip(
            #     tau, -Sim2simCfg.robot_config.tau_limit, Sim2simCfg.robot_config.tau_limit
            # )
            # import ipdb;ipdb.set_trace();
            data.ctrl = tau
            # print("kp", Sim2simCfg.robot_config.kps)
            # print("kds", Sim2simCfg.robot_config.kds)

            # data.qpos[-env.num_actions:] = motion_loader_mj.get_dof_pos_batch(phase=torch.Tensor([0]))[0].cpu().numpy()
            # data.qpos[0:3] = motion_loader_mj.get_root_trans_batch(phase=torch.Tensor([0]))[0].cpu().numpy()
            # data.qpos[3:7] = motion_loader_mj.get_root_rot_batch(phase=torch.Tensor([0]))[0].cpu().numpy()
            # data.qvel[:] = np.zeros(29, dtype=np.float32)#motion_loader_mj.get_root_rot_batch(phase=torch.Tensor([0]))[0].cpu().numpy()

            # mujoco.mj_step(model, data)
            
            mujoco.mj_step(model, data)
            count_lowlevel += 1
            
            end_time = time.time()
            # print("循环用时", end_time-step_start)
            # if count_lowlevel % Sim2simCfg.sim_config.decimation == 0:
            # viewer.cam.lookat[:] = view_center
                
            viewer.sync()
            # count_lowlevel+=1
            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    run_mujoco()