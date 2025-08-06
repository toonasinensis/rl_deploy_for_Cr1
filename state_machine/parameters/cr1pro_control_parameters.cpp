#include "control_parameters.h"
// #include <tinyxml2.h>  // 用于解析URDF
// 0 waist_z_joint
// 1 waist_x_joint
// 2 waist_y_joint
// 3 left_shoulder_y_joint
// 4 left_shoulder_x_joint
// 5 left_shoulder_z_joint
// 6 left_elbow_joint
// 7 left_wrist_z_joint
// 8 left_wrist_y_joint
// 9 left_wrist_x_joint
// 10 right_shoulder_y_joint
// 11 right_shoulder_x_joint
// 12 right_shoulder_z_joint
// 13 right_elbow_joint
// 14 right_wrist_z_joint
// 15 right_wrist_y_joint
// 16 right_wrist_x_joint
// 17 left_hip_y_joint
// 18 left_hip_x_joint
// 19 left_hip_z_joint
// 20 left_knee_joint
// 21 left_ankle_y_joint
// 22 left_ankle_x_joint
// 23 right_hip_y_joint
// 24 right_hip_x_joint
// 25 right_hip_z_joint
// 26 right_knee_joint
// 27 right_ankle_y_joint
// 28 right_ankle_x_joint
void ControlParameters::GenerateCR1BParameters(){//b样机只有臂，腰，腿，手腕
    dof_num = 29;

    arm_joint_lower_ = VecXf::Zero(7);
    arm_joint_upper_ = VecXf::Zero(7);
    joint_vel_limit_ = VecXf::Zero(7);
    torque_limit_ = VecXf::Zero(7);
    arm_kp = VecXf::Zero(7);
    arm_kd = VecXf::Zero(7);
    waist_kp = VecXf::Zero(3);
    waist_kd = VecXf::Zero(3);
    arm_link_len_ = VecXf::Zero(6);
    waist_joint_upper_ = VecXf::Zero(3);
    waist_joint_lower_ = VecXf::Zero(3);
    joint_kp = VecXf::Zero(29);
    joint_kd = VecXf::Zero(29);
    default_joint_pos = VecXf(29);


    // HIP_Y
    default_joint_pos(17) = -0.20;
    default_joint_pos(23) = -0.20;

    // KNEE
    default_joint_pos(20) = 0.42;
    default_joint_pos(26) = 0.42;

    // ANKLE_Y
    default_joint_pos(21) = -0.23;
    default_joint_pos(27) = -0.23;

    // ELBOW
    default_joint_pos(6) = 0.87;
    default_joint_pos(13) = 0.87;

    // SHOULDER_X / Y
    default_joint_pos(4) = 0.18;   // left_shoulder_x
    default_joint_pos(3) = 0.35;   // left_shoulder_y
    default_joint_pos(11) = -0.18; // right_shoulder_x
    default_joint_pos(10) = 0.35;  // right_shoulder_y


    arm_joint_lower_ << -1.5706, -3.3158, -2.8621, -1.8325, -2.8621, -1.6579, -1.6579;
    arm_joint_upper_ << 3.3158, 0.4363, 2.8621, 2.1815, 2.8621, 1.6579, 1.6579;
    //---待核对---//
    joint_vel_limit_ << 20., 20., 20., 20., 20., 20., 20.;//待核对
    torque_limit_ << 80., 80., 80., 80., 30., 30., 30.;

    //---待核对---//
    arm_kp << 80., 80., 80., 60., 200., 200., 200.;
    arm_kd << 2., 2., 2., 1.5, 5.,5., 5.;
    arm_link_len_ << 0.10135, 0.1486 , 0.1164, 0.099, 0.052, 0.082;
    hand_link_len_ = 0.15;

    joint_vel_limit_ = VecXf::Zero(6);
    torque_limit_ = VecXf::Zero(6);
    
    leg_joint_lower_ << -2.793, -0.4363, -0.5236, -2.5307, -0.7854, -0.6109;
    leg_joint_upper_ << 2.793, 2.618, 3.6652, 0.1745, 1.0472, 0.6109;

   //  z   <limit lower="-3.6652" upper="1.7453" effort="107" velocity="19.38"/>
   //  x   <limit lower="-0.7854" upper="0.7854" effort="107" velocity="19.38"/>
   //  y   <limit lower="-0.5236" upper="1.5708" effort="413" velocity="20"/>

    waist_joint_lower_ << -3.6652, -0.7854, -0.5236;
    waist_joint_upper_ << 1.7453, 0.7854,  1.5708;
    waist_kp <<120,120,120;
    waist_kd << 3,3,3;
    joint_vel_limit_ << 20., 20., 20., 20., 20., 20.;
    torque_limit_ << 413., 107., 107., 413., 107., 31.;

    leg_kp << 150., 150., 100., 150., 30., 30.;
    leg_kd << 3.75, 3.75, 2.5, 3.75, 2.5, 1. ;
    leg_link_len_ << 0.0483, 0.1635 , 0.2565, 0.410, 0.04665, 0.0; //最后一段待测

    // leg_kp = leg_kp *1.5;
    leg_kp << 150., 150., 100., 150., 100., 30.;

    joint_kp << waist_kp, arm_kp, arm_kp, leg_kp, leg_kp;
    joint_kd << waist_kd, arm_kd, arm_kd, leg_kd, leg_kd; 
        
    // joint_kp = joint_kp*1.5;

    common_policy_path_ = GetAbsPath()+"/../policy/tmppolicy.onnx";

}


// void ControlParameters::GenerateCR1BParametersFromURDF(const std::string& urdf_file) {
//     // 1. 解析URDF文件
//     ParseURDF(urdf_file);
    
//     // 2. 初始化其他参数（保持原有逻辑）
//     hand_link_len_ = 0.15;
//     // ... 其他固定参数设置
// }

// void ControlParameters::ParseURDF(const std::string& urdf_file) {
//     tinyxml2::XMLDocument doc;
//     if (doc.LoadFile(urdf_file.c_str()) != tinyxml2::XML_SUCCESS) {
//         throw std::runtime_error("Failed to load URDF file");
//     }

//     // 清空现有数据
//     // joint_names.clear();
//     std::vector<float> lower_limits, upper_limits, vel_limits, effort_limits;

//     // 遍历所有joint标签
//     tinyxml2::XMLElement* robot = doc.FirstChildElement("robot");
//     for (tinyxml2::XMLElement* joint = robot->FirstChildElement("joint"); 
//          joint != nullptr; 
//          joint = joint->NextSiblingElement("joint")) {
        
//         // // 获取joint名称
//         // const char* name = joint->Attribute("name");
//         // if (!name) continue;
//         // joint_names.push_back(name);

//         // 获取limit标签
//         tinyxml2::XMLElement* limit = joint->FirstChildElement("limit");
//         if (limit) {
//             lower_limits.push_back(limit->FloatAttribute("lower"));
//             upper_limits.push_back(limit->FloatAttribute("upper"));
//             vel_limits.push_back(limit->FloatAttribute("velocity", 0.0f));  // 默认值0
//             effort_limits.push_back(limit->FloatAttribute("effort", 0.0f)); // 默认值0
//         } else {
//             // 如果没有limit标签，使用默认值
//             lower_limits.push_back(0.0f);
//             upper_limits.push_back(0.0f);
//             vel_limits.push_back(0.0f);
//             effort_limits.push_back(0.0f);
//         }
//     }

//     // 3. 转换为Eigen向量
//     // dof_num = joint_names.size();
//     joint_lower_ = Eigen::Map<Eigen::VectorXf>(lower_limits.data(), lower_limits.size());
//     joint_upper_ = Eigen::Map<Eigen::VectorXf>(upper_limits.data(), upper_limits.size());
//     joint_vel_limit_ = Eigen::Map<Eigen::VectorXf>(vel_limits.data(), vel_limits.size());
//     torque_limit_ = Eigen::Map<Eigen::VectorXf>(effort_limits.data(), effort_limits.size());

//     std::cout<<"joint_lower_"<<joint_lower_.transpose()<<std::endl;
//     // 4. 对特定关节组进行分组（臂、腿、腰等）
//     // 这里需要根据你的URDF中关节命名规则来实现
//     // 示例伪代码：
//     /*
//     for (size_t i = 0; i < joint_names.size(); ++i) {
//         if (joint_names[i].find("arm") != std::string::npos) {
//             // 添加到arm组
//         } 
//         else if (joint_names[i].find("leg") != std::string::npos) {
//             // 添加到leg组
//         }
//         // ...
//     }
//     */
// }