import mujoco
import numpy as np
 
def print_joint_info(model_path):
    # 加载模型
    model = mujoco.MjModel.from_xml_path(model_path)
    print(f"成功加载模型: {model_path}")
    
    # 获取关节数量
    n_joints = model.njnt
    
    if n_joints == 0:
        print("模型中未找到关节。")
        return

    # 打印表头
    print("\n关节信息 (按模型定义顺序):")
    print(f"{'序号':<5} {'名称':<20} {'类型':<10} {'位置 (x, y, z)':<25} {'轴 (x, y, z)' if model.jnt_axis.size > 0 else ''}")
    print("-" * 80)
    joint_list = []
    # 遍历并打印每个关节
    for i in range(n_joints):
        joint_id = i
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        print(joint_name)
        joint_list.append(joint_name)
    
    print("joint_list",joint_list)


    n_bodies = model.nbody
    if n_bodies == 0:
        print("模型中未找到 link（body）。")
        return

    print("\nLink（Body）信息:")
    print(f"{'序号':<5} {'Link 名称':<30}")
    print("-" * 40)
    
    link_list = []
    for i in range(n_bodies):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        print(f"{i:<5} {body_name:<30}")
        link_list.append(body_name)
    
    print("\nLink 名称列表:")
    print(link_list)
    

if __name__ == "__main__":
    model_path =  'B-20250522.xml'
    print_joint_info(model_path)    