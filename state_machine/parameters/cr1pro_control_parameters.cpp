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
/*
waist_z_joint             (-3.6652, 1.7453)      107
waist_x_joint             (-0.7854, 0.7854)      107
waist_y_joint             (-0.5236, 1.5708)      413
left_shoulder_y_joint     (-3.3161, 1.5708)      107
left_shoulder_x_joint     (-0.43633, 3.5779)      107
left_shoulder_z_joint     (-2.9671, 2.9671)      107
left_elbow_joint          (-0.767945, 1.74533)      107
left_wrist_z_joint        (-2.9671, 2.9671)      31
left_wrist_y_joint        (-1.5708, 1.5708)      31
left_wrist_x_joint        (-1.5708, 1.5708)      31
right_shoulder_y_joint    (-3.3161, 1.5708)      107
right_shoulder_x_joint    (-3.5779, 0.43633)      107
right_shoulder_z_joint    (-2.9671, 2.9671)      107
right_elbow_joint         (-0.767945, 1.74533)      107
right_wrist_z_joint       (-2.9671, 2.9671)      31
right_wrist_y_joint       (-1.5708, 1.5708)      31
right_wrist_x_joint       (-1.5708, 1.5708)      31
left_hip_y_joint          (-2.7925, 1.5708)      413
left_hip_x_joint          (-0.43633, 2.618)      107
left_hip_z_joint          (-0.5236, 3.6652)      107
left_knee_joint           (-0.17453, 2.5307)      413
left_ankle_y_joint        (-1.0472, 0.7854)      107
left_ankle_x_joint        (-0.61087, 0.61087)      31
right_hip_y_joint         (-2.7925, 1.5708)      413
right_hip_x_joint         (-2.618, 0.43633)      107
right_hip_z_joint         (-3.6652, 0.5236)      107
right_knee_joint          (-0.17453, 2.5307)      413
right_ankle_y_joint       (-1.0472, 0.7854)      107
right_ankle_x_joint       (-0.61087, 0.61087)      31

###
*/
#include "json.hpp"
struct JointPIDGains {
    std::vector<double> kp;
    std::vector<double> kd;
};
using json = nlohmann::json;

// Load JSON from file
json load_json(const std::string& filepath) {
    std::ifstream f(filepath);
    if (!f.is_open()) {
        throw std::runtime_error("Could not open file: " + filepath);
    }
    return json::parse(f);
}

void ControlParameters::GenerateCR1PROParameters(){//b样机只有臂，腰，腿，手腕
   
    json data_json = load_json("../config/config.json");

    auto kp_waist_json = data_json["pid_gains"]["waist"]["kp"].get<std::vector<float>>();
    auto kp_arm_json = data_json["pid_gains"]["arm"]["kp"].get<std::vector<float>>();
    auto kp_leg_json = data_json["pid_gains"]["leg"]["kp"].get<std::vector<float>>();

    auto kd_waist_json = data_json["pid_gains"]["waist"]["kd"].get<std::vector<float>>();
    auto kd_arm_json = data_json["pid_gains"]["arm"]["kd"].get<std::vector<float>>();
    auto kd_leg_json = data_json["pid_gains"]["leg"]["kd"].get<std::vector<float>>();


    // std::cout<<"kp_waist"<<kp_waist<<std::endl;
   
    dof_num_ = 31;
    waist_dof_num_ = 3;
    arm_dof_num_ = 7;
    leg_dof_num_ = 6;
    neck_dof_num_ = 2;

    //alloacte memeory
    waist_kp = VecXf::Zero(waist_dof_num_);
    waist_kd = VecXf::Zero(waist_dof_num_);
    arm_kp = VecXf::Zero(arm_dof_num_);
    arm_kd = VecXf::Zero(arm_dof_num_);
    neck_kp =  VecXf::Zero(neck_dof_num_);
    neck_kd =  VecXf::Zero(neck_dof_num_);

    leg_kd = VecXf::Zero(leg_dof_num_);
    leg_kp = VecXf::Zero(leg_dof_num_);

      waist_kp_pc =  VecXf::Zero(waist_dof_num_);
      waist_kd_pc =  VecXf::Zero(waist_dof_num_);

      arm_kp_pc = VecXf::Zero(arm_dof_num_);
      arm_kd_pc = VecXf::Zero(arm_dof_num_);

      leg_kp_pc = VecXf::Zero(leg_dof_num_);
      leg_kd_pc =  VecXf::Zero(leg_dof_num_);

      neck_kp_pc = VecXf::Zero(neck_dof_num_);
      neck_kd_pc = VecXf::Zero(neck_dof_num_);


    joint_kp = VecXf::Zero(dof_num_);
    joint_kd = VecXf::Zero(dof_num_);

    waist_joint_lower_ = VecXf::Zero(waist_dof_num_);
    waist_joint_upper_ = VecXf::Zero(waist_dof_num_);
    arm_joint_lower_ = VecXf::Zero(arm_dof_num_);
    arm_joint_upper_ = VecXf::Zero(arm_dof_num_);

    leg_joint_upper_ = VecXf::Zero(leg_dof_num_);
    leg_joint_lower_ = VecXf::Zero(leg_dof_num_);

    neck_joint_upper_ = VecXf::Zero(neck_dof_num_);
    neck_joint_lower_ = VecXf::Zero(neck_dof_num_);

    

    joint_vel_limit_ = VecXf::Zero(dof_num_);
    torque_limit_ = VecXf::Zero(dof_num_);


    default_joint_pos = VecXf::Zero(dof_num_);

    //this is for rl,but the squence is robot
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

 

    //upper and lower
    waist_joint_lower_ << -3.6652, -0.7854, -0.5236;
    waist_joint_upper_ << 1.7453, 0.7854, 1.5708;

    arm_joint_lower_ << -3.3161, -0.43633, -2.9671, -0.767945, -2.9671, -1.5708, -1.5708;//注意左右shoulderx,shoulderz,wristz,wristx需镜像
    arm_joint_upper_ << 1.5708,  3.5799,   2.9671,  1.74533,   2.9671,  1.5708,  1.5708;

    leg_joint_lower_ << -2.7925, -0.4363, -0.5236, -0.1745, -1.0472, -0.61087;//注意左右hipx,hipz,anklex需镜像
    leg_joint_upper_ << 1.5708,  2.618,   3.6652,  2.5307,  0.7854,  0.61087;


    neck_joint_lower_ << -1., -1.;
    neck_joint_upper_ << 1., 1.;
    //limit: 

    joint_vel_limit_ << 19.38, 19.38, 20.,
                        19.38, 19.38, 19.38, 19.38, 23.76, 23.76, 23.76,
                        19.38, 19.38, 19.38, 19.38, 23.76, 23.76, 23.76,
                        20., 19.38, 19.38, 20., 19.38, 23.76, 
                        20., 19.38, 19.38, 20., 19.38, 23.76, 
                        5., 5.;

    torque_limit_ <<    107., 107., 413.,
                        107., 107., 107., 107., 31., 31., 31.,
                        107., 107., 107., 107., 31., 31., 31.,
                        413., 107., 107., 413., 107., 31., 
                        413., 107., 107., 413., 107., 31.,
                        5., 5.;


    //pd参数:
     
    waist_kp <<120,120,120;
    waist_kd << 3,3,3;

    arm_kp << 80., 80., 80., 60., 90., 90., 90.;
    arm_kd << 2., 2., 2., 1.5, 2,2.,2;
   

    leg_kp << 150., 150., 100., 150., 100., 30.;
    leg_kd << 3.75, 3.75, 2.5, 3.75, 2.5, 1. ;

    neck_kp  <<0.,0.;
    neck_kd  <<0.,0.;

    //read from json:

    for (int i=0 ;i<waist_dof_num_;i++)
    {
      waist_kp(i) = kp_waist_json.at(i);
      waist_kd(i) = kd_waist_json.at(i);
    }
    
    for (int i=0 ;i<arm_dof_num_;i++)
    {
      arm_kp(i) = kp_arm_json.at(i);
      arm_kd(i) = kd_arm_json.at(i);
    }
    for (int i=0 ;i<leg_dof_num_;i++)
    {
      leg_kp(i) = kp_leg_json.at(i);
      leg_kd(i) = kd_leg_json.at(i);
    }

    float kp_scale  = data_json["kp_scale"].get<float>();
    std::cout << "waist_kd:\n" << waist_kd.transpose() << std::endl;
    std::cout << "arm_kp:\n" << arm_kp.transpose() << std::endl;
    std::cout << "leg_kp:\n" << leg_kp.transpose() << std::endl;
    std::cout << "neck_kp:\n" << neck_kp.transpose() << std::endl;
    

    //this is for 恢复0位
    //temp small
    
    waist_kp_pc << 300., 300., 600.;
    waist_kd_pc << 3., 3., 6.;


    arm_kp_pc << 600., 600., 600., 600., 90., 90., 90.;
    arm_kd_pc << 6., 6., 6., 6., 2., 2., 2.;

    leg_kp_pc << 1800.0, 600.0, 600.0, 1800.0, 600.0, 90.0;
    leg_kd_pc << 24.0, 6.0, 6.0, 24.0, 6.0, 2.0;


    neck_kp_pc << 0., 0.;
    neck_kd_pc << 0., 0.;

    joint_kp << waist_kp, arm_kp, arm_kp,leg_kp, leg_kp, neck_kp;
    joint_kd << waist_kd, arm_kd, arm_kd,leg_kd, leg_kd, neck_kd; 
    joint_kp = joint_kp*kp_scale;

    std::cout << "rl's joint_kp:\n" << joint_kp.transpose() << std::endl;


    common_policy_path_ = GetAbsPath()+"/../policy/tmppolicy.onnx";

}

 