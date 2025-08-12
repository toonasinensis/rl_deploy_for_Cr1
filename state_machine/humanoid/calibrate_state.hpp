/**
 * @file standup_state.hpp
 * @brief from sit state to stand state
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef HUMANOID_CALIBRATE_STATE_HPP_
#define HUMANOID_CALIBRATE_STATE_HPP_

#include "state_base.h"
#include "json.hpp"
using json = nlohmann::json;

namespace humanoid{
class CalibrateState : public StateBase{
private:
    VecXf init_joint_pos_, init_joint_vel_, current_joint_pos_, current_joint_vel_;
    float time_stamp_record_, run_time_;

    std::vector<std::string> joint_order = {
    "waist_z_joint", "waist_x_joint", "waist_y_joint",
    "left_shoulder_y_joint", "left_shoulder_x_joint", "left_shoulder_z_joint",
    "left_elbow_joint", "left_wrist_z_joint", "left_wrist_y_joint", "left_wrist_x_joint",
    "right_shoulder_y_joint", "right_shoulder_x_joint", "right_shoulder_z_joint",
    "right_elbow_joint", "right_wrist_z_joint", "right_wrist_y_joint", "right_wrist_x_joint",
    "left_hip_y_joint", "left_hip_x_joint", "left_hip_z_joint",
    "left_knee_joint", "left_ankle_y_joint", "left_ankle_x_joint",
    "right_hip_y_joint", "right_hip_x_joint", "right_hip_z_joint",
    "right_knee_joint", "right_ankle_y_joint", "right_ankle_x_joint","neck1","neck2"
    };

    VecXf kp_ = VecXf::Zero(cp_ptr_->dof_num_);
    VecXf kd_ = VecXf::Zero(cp_ptr_->dof_num_);    
    float stand_duration_ = 2.;
    int target_joint_now = 0;
    int target_joint_last = 0;

    VecXf planning_joint_pos = VecXf::Zero(cp_ptr_->dof_num_);
    VecXf planning_joint_vel = VecXf::Zero(cp_ptr_->dof_num_);
    VecXf planning_joint_tau = VecXf::Zero(cp_ptr_->dof_num_);

    MatXf joint_cmd_ = MatXf::Zero(cp_ptr_->dof_num_, 5);
    VecXf goal_joint_pos_ = VecXf::Zero(cp_ptr_->dof_num_);

    void GetRobotJointValue(){
        current_joint_pos_ = ri_ptr_->GetJointPosition();
        current_joint_vel_ = ri_ptr_->GetJointVelocity();
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
    }

    void RecordJointData(){
        init_joint_pos_ = current_joint_pos_;
        init_joint_vel_ = current_joint_vel_;
        time_stamp_record_ = run_time_;
    }

    float GetCubicSplinePos(float x0, float v0, float xf, float vf, float t, float T){
        if(t >= T) return xf;
        float a, b, c, d;
        d = x0;
        c = v0;
        a = (vf*T - 2*xf + v0*T + 2*x0) / pow(T, 3);
        b = (3*xf - vf*T - 2*v0*T - 3*x0) / pow(T, 2);
        return a*pow(t, 3)+b*pow(t, 2)+c*t+d;
    }
    float GetCubicSplineVel(float x0, float v0, float xf, float vf, float t, float T){
        if(t >= T) return 0;
        float a, b, c;
        c = v0;
        a = (vf*T - 2*xf + v0*T + 2*x0) / pow(T, 3);
        b = (3*xf - vf*T - 2*v0*T - 3*x0) / pow(T, 2);
        return 3.*a*pow(t, 2) + 2.*b*t + c;
    }

public:
    CalibrateState(const RobotName& robot_name, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_name, state_name, data_ptr){
            stand_duration_ = cp_ptr_->stand_duration_;
            
        }
    ~CalibrateState(){}

    virtual void OnEnter() {

        std::ifstream input_file("../config/config.json");
        json config;
        input_file>>config;
        if(!input_file.is_open()){
            std::cerr << "Failed to open config.json,add an empty config.json!" << std::endl;
            
        }
        // std::vector<float> save_offset_pos(goal_joint_pos_.data(),goal_joint_pos_.data()+goal_joint_pos_.size());
        std::vector<float> saved_offet_pos = config["offset_data"].get<std::vector<float>>();
        
        // for (int i=0 ;i<cp_ptr_->dof_num_;i++)
        // {
        //     goal_joint_pos_(i) = saved_offet_pos.at(i);
        // }
        
        std::cout<<"read data"<<std::endl;

        GetRobotJointValue();
        RecordJointData();
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::CalibrateMode);
        
        
    };
    virtual void OnExit() {
    }
    virtual void Run() {
        GetRobotJointValue();        
       

        target_joint_now = uc_ptr_->GetUserCommand()->target_joint;
        if (target_joint_last != target_joint_now)
        {
            std::cout<<"new joint calibrate joint is "<<target_joint_now<<" "<<joint_order.at(target_joint_now)<<std::endl;
            target_joint_last = target_joint_now;
            // uc_ptr_->GetUserCommand()->calibrateJoint = 0;
        }
        
        goal_joint_pos_(target_joint_now) += uc_ptr_->GetUserCommand()->calibrateJoint;

        if (uc_ptr_->GetUserCommand()->finish_calibrate)
        {
        std::ifstream input_file("../config/config.json");
        json config;
        input_file>>config;
        std::vector<float> save_offset_pos(goal_joint_pos_.data(),goal_joint_pos_.data()+goal_joint_pos_.size());
        
        std::vector<float> saved_offet_pos = config["offset_data"].get<std::vector<float>>();

        for (int i=0;i<cp_ptr_->dof_num_;i++)//increment
        {
            saved_offet_pos.at(i) += save_offset_pos.at(i);
        }
        config["offset_data"] = saved_offet_pos;
        std::ofstream output_file("../config/config.json");

        output_file<<config.dump(4);
        std::cout<<"save data"<<std::endl;
        uc_ptr_->GetUserCommand()->finish_calibrate = false;
        }

        uc_ptr_->GetUserCommand()->calibrateJoint = 0;
        //-------------------关节测试-----------------//
        float init_time = 3.;
        float t = run_time_ - time_stamp_record_-init_time;
        if(run_time_ - time_stamp_record_ <= init_time){
            for(int i=0;i<current_joint_pos_.rows();++i){
                planning_joint_pos(i) = GetCubicSplinePos(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0, 
                                                run_time_ - time_stamp_record_, init_time);
                planning_joint_vel(i) = GetCubicSplineVel(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0, 
                                                run_time_ - time_stamp_record_, init_time);
        }
        }else{
            for(int i=0;i<current_joint_pos_.rows();++i){planning_joint_pos(i) = goal_joint_pos_(i); planning_joint_vel.setZero();}
        }

      
 
        //-------------------关节测试-----------------//
        if(cp_ptr_->dof_num_ == 31) {
            kp_<<cp_ptr_->waist_kp_pc, cp_ptr_->arm_kp_pc, cp_ptr_->arm_kp_pc, cp_ptr_->leg_kp_pc, cp_ptr_->leg_kp_pc, cp_ptr_->neck_kp_pc;
            kd_<<cp_ptr_->waist_kd_pc, cp_ptr_->arm_kd_pc, cp_ptr_->arm_kd_pc, cp_ptr_->leg_kd_pc, cp_ptr_->leg_kd_pc, cp_ptr_->neck_kd_pc;
        }else if(cp_ptr_->dof_num_ == 21) {
            kp_<<cp_ptr_->waist_kp_pc,cp_ptr_->arm_kp_pc,cp_ptr_->arm_kp_pc, cp_ptr_->leg_kp_pc,cp_ptr_->leg_kp_pc;
            kd_<<cp_ptr_->waist_kd_pc,cp_ptr_->arm_kd_pc,cp_ptr_->arm_kd_pc, cp_ptr_->leg_kd_pc,cp_ptr_->leg_kd_pc;
        }else{
            kp_<<cp_ptr_->leg_kp_pc,cp_ptr_->leg_kp_pc;
            kd_<<cp_ptr_->leg_kd_pc,cp_ptr_->leg_kd_pc;            
        }


        // std::cout<<"---------info-----------"<<std::endl;
        // std::cout<<kp_.transpose()<<std::endl;
        // std::cout<<kd_.transpose()<<std::endl;
        // std::cout<<amplitude_.transpose()<<std::endl;
        // std::cout<<planning_joint_vel.transpose()<<std::endl;
        // std::cout<<"--------info end---------"<<std::endl;

        joint_cmd_.col(0) = kp_;
        joint_cmd_.col(2) = kd_;  
        joint_cmd_.col(1) = planning_joint_pos;
        joint_cmd_.col(3) = planning_joint_vel;
        joint_cmd_.col(4) = planning_joint_tau;

        ri_ptr_->SetJointCommand(joint_cmd_);
    }
    virtual bool LoseControlJudge() {
        if(uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::JointDamping)) return true;
        return false;
    }
    virtual StateName GetNextStateName() {
        if(uc_ptr_->GetUserCommand()->safe_control_mode!=0){
            std::cout << "safe_control_mode:" << uc_ptr_->GetUserCommand()->safe_control_mode << std::endl;
            return StateName::kJointDamping;
        }
        if(uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::StandingUp)) return StateName::kStandUp;
        if(uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::WaitingForStand)) return StateName::kIdle;
        if(uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::RLControlMode)) return StateName::kRLControl;
        return StateName::kCalibrateMode;
    }
};

};//namespace humanoid

#endif