#include "control_parameters.h"
// 0 waist_z_joint
// 1 left_shoulder_y_joint
// 2 left_shoulder_x_joint
// 3 left_shoulder_z_joint
// 4 left_elbow_joint
// 5 right_shoulder_y_joint
// 6 right_shoulder_x_joint
// 7 right_shoulder_z_joint
// 8 right_elbow_joint
// 9 left_hip_y_joint
// 10 left_hip_x_joint
// 11 left_hip_z_joint
// 12 left_knee_joint
// 13 left_ankle_y_joint
// 14 left_ankle_x_joint
// 15 right_hip_y_joint
// 16 right_hip_x_joint
// 17 right_hip_z_joint
// 18 right_knee_joint
// 19 right_ankle_y_joint
// 20 right_ankle_x_joint
/* 

###
*/
using json = nlohmann::json;
void ControlParameters::GenerateCR1STDParameters(){//b样机只有臂，腰，腿，手腕

    std::string filepath = "../config/config.json";  
    std::ifstream f(filepath);
    if (!f.is_open()) {
        throw std::runtime_error("Could not open file: " + filepath);
      }

    json data_json = json::parse(f);

    auto kp_waist_json = data_json["pid_gains"]["waist"]["kp"].get<std::vector<float>>();
    auto kp_arm_json = data_json["pid_gains"]["arm"]["kp"].get<std::vector<float>>();
    auto kp_leg_json = data_json["pid_gains"]["leg"]["kp"].get<std::vector<float>>();

    auto kd_waist_json = data_json["pid_gains"]["waist"]["kd"].get<std::vector<float>>();
    auto kd_arm_json = data_json["pid_gains"]["arm"]["kd"].get<std::vector<float>>();
    auto kd_leg_json = data_json["pid_gains"]["leg"]["kd"].get<std::vector<float>>();



    default_joint_pos = VecXf::Zero(dof_num_);

    //this is for rl,but the squence is robot
    // HIP_Y
    default_joint_pos(9) = -0.20;
    default_joint_pos(15) = -0.20;

    // KNEE
    default_joint_pos(12) = 0.42;
    default_joint_pos(18) = 0.42;

    // ANKLE_Y
    default_joint_pos(13) = -0.23;
    default_joint_pos(19) = -0.23;

    // ELBOW
    default_joint_pos(4) = 0.87;
    default_joint_pos(8) = 0.87;

    // SHOULDER_X / Y
    default_joint_pos(2) = 0.18;   // left_shoulder_x
    default_joint_pos(1) = 0.35;   // left_shoulder_y
    default_joint_pos(6) = -0.18; // right_shoulder_x                  
    default_joint_pos(5) = 0.35;  // right_shoulder_y


 


    // std::cout<<"kp_waist"<<kp_waist<<std::endl;
   
    dof_num_ = 21;
    waist_dof_num_ = 1;
    arm_dof_num_ = 4;
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


    //upper and lower
    waist_joint_lower_ << -3.6652;
    waist_joint_upper_ << 1.7453;

    arm_joint_lower_ << -3.3161, -0.43633, -2.9671, -0.767945;//注意左右shoulderx,shoulderz,wristz,wristx需镜像
    arm_joint_upper_ << 1.5708,  3.5799,   2.9671,  1.74533;

    leg_joint_lower_ << -2.7925, -0.4363, -0.5236, -0.1745, -1.0472, -0.61087;//注意左右hipx,hipz,anklex需镜像
    leg_joint_upper_ << 1.5708,  2.618,   3.6652,  2.5307,  0.7854,  0.61087;


    neck_joint_lower_ << -1., -1.;
    neck_joint_upper_ << 1., 1.;
    //limit: 

    joint_vel_limit_ << 19.38,
                        19.38, 19.38, 19.38, 19.38,
                        19.38, 19.38, 19.38, 19.38,
                        20., 19.38, 19.38, 20., 19.38, 23.76, 
                        20., 19.38, 19.38, 20., 19.38, 23.76, 
                        5., 5.;

    torque_limit_ <<    107.,
                        107., 107., 107., 
                        107., 107., 107., 
                        413., 107., 107., 413., 107., 31., 
                        413., 107., 107., 413., 107., 31.,
                        5., 5.;


    //pd参数:
     
    waist_kp <<120;
    waist_kd << 3;

    arm_kp << 80., 80., 80., 60.;
    arm_kd << 2., 2., 2., 1.5;
   

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
    
    waist_kp_pc << 300.;
    waist_kd_pc << 3.;


    arm_kp_pc << 600., 600., 600., 600.;
    arm_kd_pc << 6., 6., 6., 6.;

    leg_kp_pc << 1800.0, 600.0, 600.0, 1800.0, 600.0, 90.0;
    leg_kd_pc << 24.0, 6.0, 6.0, 24.0, 6.0, 2.0;


    neck_kp_pc << 0., 0.;
    neck_kd_pc << 0., 0.;

    joint_kp << waist_kp, arm_kp, arm_kp, leg_kp, leg_kp;
    joint_kd << waist_kd, arm_kd, arm_kd, leg_kd, leg_kd; 
    joint_kp = joint_kp*kp_scale;

    std::cout << "rl's joint_kp:\n" << joint_kp.transpose() << std::endl;


    common_policy_path_ = GetAbsPath()+"/../policy/tmppolicy.onnx";

}

 