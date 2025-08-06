/**
 * @file control_parameters.h
 * @brief basic control parameters
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef CONTROL_PARAMETERS_HPP_
#define CONTROL_PARAMETERS_HPP_

#include "common_types.h"
#include "custom_types.h"

using namespace types;

class ControlParameters
{
private:
    void GenerateCR1ARMParameters();
    void GenerateCR1LEGParameters();
    void GenerateCR1Parameters();
    void GenerateCR1PROParameters();
    // void GenerateCR1BParametersFromURDF(const std::string& urdf_file);
    // void ParseURDF(const std::string& urdf_file);


public:
    ControlParameters(RobotName robot_name){
        if(robot_name==RobotName::CR1ARM) GenerateCR1ARMParameters();
        else if(robot_name==RobotName::CR1LEG) GenerateCR1LEGParameters();
        else if(robot_name==RobotName::CR1A) GenerateCR1Parameters();
        else if(robot_name==RobotName::CR1Pro) GenerateCR1PROParameters();//GenerateCR1BParametersFromURDF("/home/tian/Desktop/learn/imation/deploy/rl_deploy/simulation/urdf_model/CR01B-pro/B-20250522.urdf");
        else{
            std::cerr << "Not Deafult Robot" << std::endl;
        }
    }
    ~ControlParameters(){}

    /**
     * @brief robot dof numbers
     */
    int dof_num_ = 31;
    int waist_dof_num_ = 3;
    int arm_dof_num_ = 7;
    int leg_dof_num_ = 6;
    int neck_dof_num_ = 2;

    /**
     * @brief robot link length
     */
    float body_len_x_, body_len_y_;
    float hip_len_, thigh_len_, shank_len_;

    /**
     * @brief stand height configure
     */
    float pre_height_, stand_height_; 

    /**
     * @brief one leg joint PD gain
     */
    Vec3f swing_leg_kp_, swing_leg_kd_;

    /**
     * @brief joint position limitation
     */
    Vec3f fl_joint_lower_, fl_joint_upper_;

    /**
     * @brief joint velocity limitation
     */
    VecXf joint_vel_limit_;

    /**
     * @brief joint torque limitation
     */
    VecXf torque_limit_;
    
    /**
     * @brief stand up duration
     */
    float stand_duration_ = 1.5;

    /**
     * @brief lie down duration
     */
    float liedown_duration_ = 2.0;

    /**
     * @brief wheel vel limit for wheel-legged robot
     */
    float wheel_vel_limit_ = 100;
    
    /**
     * @brief wheel link length to shank
     */
    float wheel_link_len_ = 0.040575;

    /**
     * @brief arm joint position limitation
     */
    VecXf arm_joint_lower_, arm_joint_upper_;

    /**
     * @brief arm joint kp
     */
    VecXf arm_kp;

    /**
     * @brief arm joint kd
     */
    VecXf arm_kd;

    /**
     * @brief arm link length
     */
    VecXf arm_link_len_;

    /**
     * @brief hand link length
     */
    float hand_link_len_;

    /**
     * @brief human leg joint position limitation
     */
    Vec6f leg_joint_lower_, leg_joint_upper_;

    /**
     * @brief human leg joint kp
     */
    Vec6f leg_kp;

    /**
     * @brief human leg joint kd
     */
    Vec6f leg_kd;
 

    /**
     * @brief human leg link length
     */
    Vec6f leg_link_len_;

    /**
     * @brief human waist joint position limitation
     */
    VecXf waist_joint_lower_, waist_joint_upper_;

    /**
     * @brief human waist joint position limitation
     */
    VecXf joint_lower_, joint_upper_;

// joint_names
    /**
     * @brief human waist joint kp
     */
    VecXf waist_kp;

    /**
     * @brief human waist joint kd
     */
    VecXf waist_kd;
    /**
 * @brief human waist joint kp
 */
    //this is for pos ctrl esplically when init from default
    VecXf waist_kp_pc;
    VecXf waist_kd_pc;

    VecXf arm_kp_pc;
    VecXf arm_kd_pc;

    VecXf leg_kp_pc;
    VecXf leg_kd_pc;

    VecXf neck_kp_pc;
    VecXf neck_kd_pc;

    // joint_names
    /**
     * @brief human all joint kp
     */
    VecXf joint_kp;

    /**
     * @brief human all default joint
     */
    VecXf default_joint_pos;

 
    /**
     * @brief human all joint kd
     */
    VecXf joint_kd;
     /**
     * @brief human wrist joint kp
     */
    VecXf wrist_kp;

    /**
     * @brief human wrist joint kd
     */
    VecXf wrist_kd;


    /**
     * @brief human neck joint position limitation
     */
    VecXf neck_joint_lower_, neck_joint_upper_;

    /**
     * @brief human neck joint kp
     */
    VecXf neck_kp;

    /**
     * @brief human neck joint kd
     */
    VecXf neck_kd;
    /**
     * @brief policy path
     */
    std::string common_policy_path_;
    Vec3f common_policy_p_gain_, common_policy_d_gain_;
    Vec6f humanleg_policy_p_gain_, humanleg_policy_d_gain_;

    // std::string speed_policy_path_;
    // Vec3f speed_policy_p_gain_, speed_policy_d_gain_;

    // std::string tumbler_policy_path_;
    // Vec3f tumbler_policy_p_gain_, tumbler_policy_d_gain_;
};



#endif