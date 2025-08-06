#include "control_parameters.h"

void ControlParameters::GenerateCR1LEGParameters(){
    dof_num = 12;

    joint_vel_limit_ = VecXf::Zero(6);
    torque_limit_ = VecXf::Zero(6);

    leg_joint_lower_ << -2.793, -0.4363, -0.5236, -0.1745, -1.0472, -0.7853;//A样机 ankleroll范围跟安装有关系
    leg_joint_upper_ << 1.57, 1.57, 3.6652, 2.5307, 0.7854, 0.7853;

//    leg_joint_lower_ << -2.793, -0.4363, -0.5236, -0.1745, -0.7854, -0.6109;//B样机
//     leg_joint_upper_ << 2.793, 2.618, 3.6652, 2.5307, 1.0472, 0.6109;

    joint_vel_limit_ << 19.5, 19.38, 19.38, 19.5, 19.38, 23.76;
    torque_limit_ << 413., 107., 107., 413., 107., 31.;

    leg_kp << 1800.0, 600.0, 600.0, 1800.0, 600.0, 90.0;
    leg_kd << 12.0, 6.0, 6.0, 12.0, 6.0, 2.0;
    leg_link_len_ << 0.0483, 0.1635 , 0.2565, 0.410, 0.04665, 0.0;//最后一段待测

    pre_height_ = 0.40;
    stand_height_ = 0.75;
    stand_duration_ = 4.0;
    liedown_duration_ = 5.;

    common_policy_path_ = GetAbsPath()+"/../policy/cr1leg_test_3.onnx";
    humanleg_policy_p_gain_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    humanleg_policy_d_gain_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
}