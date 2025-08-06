#include "control_parameters.h"

void ControlParameters::GenerateCR1Parameters(){//A样机只有臂和腿
    dof_num_ = 26;

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
    arm_kp << 200., 200., 200., 200., 30., 30., 30.;
    arm_kd << 4., 4., 4., 4., 1., 1., 1.;
    arm_link_len_ << 0.10135, 0.1486 , 0.1164, 0.099, 0.052, 0.082;
    hand_link_len_ = 0.15;

    joint_vel_limit_ = VecXf::Zero(6);
    torque_limit_ = VecXf::Zero(6);
    
    leg_joint_lower_ << -2.793, -0.4363, -0.5236, -2.5307, -0.7854, -0.6109;
    leg_joint_upper_ << 2.793, 2.618, 3.6652, 0.1745, 1.0472, 0.6109;

    joint_vel_limit_ << 20., 20., 20., 20., 20., 20.;
    torque_limit_ << 413., 107., 107., 413., 107., 31.;

    leg_kp << 200., 200., 200., 200., 30., 30., 30.;
    leg_kd << 4., 4., 4., 4., 1., 1., 1.;
    leg_link_len_ << 0.0483, 0.1635 , 0.2565, 0.410, 0.04665, 0.0;//最后一段待测
}