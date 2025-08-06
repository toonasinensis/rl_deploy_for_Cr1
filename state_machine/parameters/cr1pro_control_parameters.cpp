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
void ControlParameters::GenerateCR1PROParameters(){//b样机只有臂，腰，腿，手腕
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



    
    

    default_joint_pos = VecXf(dof_num_);

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
    std::cout << "joint_kp:\n" << joint_kp.transpose() << std::endl;
    std::cout << "waist_kd:\n" << waist_kd.transpose() << std::endl;
    std::cout << "arm_kp:\n" << arm_kp.transpose() << std::endl;
    std::cout << "leg_kp:\n" << leg_kp.transpose() << std::endl;
    std::cout << "neck_kp:\n" << neck_kp.transpose() << std::endl;
 

    joint_kp << waist_kp, arm_kp, arm_kp,leg_kp, leg_kp, neck_kp;
    joint_kd << waist_kd, arm_kd, arm_kd,leg_kd, leg_kd, neck_kd; 

    common_policy_path_ = GetAbsPath()+"/../policy/tmppolicy.onnx";

}

 