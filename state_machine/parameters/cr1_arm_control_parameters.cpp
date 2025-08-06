#include "control_parameters.h"

void ControlParameters::GenerateCR1ARMParameters(){
    dof_num_ = 14;

    arm_joint_lower_ = VecXf::Zero(7);
    arm_joint_upper_ = VecXf::Zero(7);
    joint_vel_limit_ = VecXf::Zero(7);
    torque_limit_ = VecXf::Zero(7);
    arm_kp = VecXf::Zero(7);
    arm_kd = VecXf::Zero(7);
    arm_link_len_ = VecXf::Zero(6);


    arm_joint_lower_ << -1.5706, -3.3158, -2.8621, -1.8325, -2.8621, -1.6579, -1.6579;
    arm_joint_upper_ << 3.3158, 0.4363, 2.8621, 2.1815, 2.8621, 1.6579, 1.6579;
    //---待核对---//
    joint_vel_limit_ << 20., 20., 20., 20., 20., 20., 20.;//待核对
    torque_limit_ << 80., 80., 80., 80., 30., 30., 30.;

    //---待核对---//
    arm_kp << 80., 80., 80., 60., 30., 30., 30.;
    arm_kd << 2., 2., 2., 1.5, 1., 1., 1.;
    arm_link_len_ << 0.10135, 0.1486 , 0.1164, 0.099, 0.052, 0.082;
    hand_link_len_ = 0.15;
}