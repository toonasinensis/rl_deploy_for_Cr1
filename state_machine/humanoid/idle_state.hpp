/**
 * @file idle_state.hpp
 * @brief robot need to confirm sensor input while in idle state
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef HUMANOID_IDLE_STATE_HPP_
#define HUMANOID_IDLE_STATE_HPP_


#include "state_base.h"

namespace humanoid{
class IdleState : public StateBase{
private:
    bool joint_normal_flag_ = false, imu_normal_flag_ = false;
    bool first_enter_flag_ = true;
    int cnt=0;
    VecXf joint_pos_, joint_vel_, joint_tau_;
    Vec3f rpy_, acc_, omg_;
    float enter_state_time_ = 10000.;

    void GetProprioceptiveData(){
        joint_pos_ = ri_ptr_->GetJointPosition();
        joint_vel_ = ri_ptr_->GetJointVelocity();
        joint_tau_ = ri_ptr_->GetJointTorque();
        rpy_ = ri_ptr_->GetImuRpy();
        acc_ = ri_ptr_->GetImuAcc();
        omg_ = ri_ptr_->GetImuOmega();
    }

    bool JointDataNormalCheck(){//TODO: temeropal unuse the check
    return true;
        VecXf joint_pos_lower(cp_ptr_->dof_num), joint_pos_upper(cp_ptr_->dof_num);
        joint_pos_lower <<cp_ptr_->waist_joint_lower_;
        joint_pos_upper << cp_ptr_->waist_joint_upper_;
        joint_pos_lower << cp_ptr_->leg_joint_lower_, cp_ptr_->leg_joint_lower_;
        joint_pos_upper << cp_ptr_->leg_joint_upper_, cp_ptr_->leg_joint_upper_;
        
        joint_pos_lower(7) = -cp_ptr_->leg_joint_upper_(1);
        joint_pos_upper(7) = -cp_ptr_->leg_joint_lower_(1);
        joint_pos_lower(8) = -cp_ptr_->leg_joint_upper_(2);
        joint_pos_upper(8) = -cp_ptr_->leg_joint_lower_(2);
        joint_pos_lower(11) = -cp_ptr_->leg_joint_upper_(5);
        joint_pos_upper(11) = -cp_ptr_->leg_joint_lower_(5);
        // std::cout<<"joint_pos_lower:"<<joint_pos_lower.transpose()<<std::endl;
        // std::cout<<"joint_pos_upper:"<<joint_pos_upper.transpose()<<std::endl;
        //---正负待根据实际安装确认---//
        std::cout<<joint_pos_.size()<<"cp_ptr_->dof_num"<<cp_ptr_->dof_num<<std::endl;
        for(int i=0;i<cp_ptr_->dof_num;++i){
            if(std::isnan(joint_pos_lower(i)) || joint_pos_(i) > joint_pos_upper(i)+0.1 || joint_pos_(i) < joint_pos_lower(i)-0.1) {
                // std::cout << "joint pos " << i << " : " << joint_pos_(i) << " | " 
                //                                         << joint_pos_lower(i) << " " << joint_pos_upper(i) << std::endl;
                // std::cout << "joint "<<i<< "is out of range! "<<std::endl<<"upper: "<<joint_pos_upper(i)+0.1<<",lower: " << joint_pos_lower(i)-0.1<<"now:"<<joint_pos_(i)<<std::endl;
                return false;
            }
            if(std::isnan(joint_vel_(i)) || (joint_vel_(i)) > cp_ptr_->joint_vel_limit_(i%7) + 0.1) {
                // std::cout << "joint vel " << i << " : " << joint_vel_(i) << " | " << cp_ptr_->joint_vel_limit_(i%3) << std::endl;
                return false;
            }
        }
        return true;
    }
    bool ImuDataNormalCheck(){
        return true;
        for(int i=0;i<3;++i){
            if(std::isnan(rpy_(i)) || fabs(rpy_(i)) > M_PI){
                std::cout << "rpy_ " << i << " : " << rpy_(i) << std::endl;
                return false;
            } 
            if(std::isnan(omg_(i)) || fabs(omg_(i)) > M_PI){
                std::cout << "omg_ " << i << " : " << omg_(i) << std::endl;
                return false;
            }
        }
        if(acc_.norm() < 0.1*gravity || acc_.norm() > 3.0*gravity){
            std::cout << "acc " << " : " << acc_.transpose() << std::endl;
            return false;
        }
        return true;
    }

    void DisplayProprioceptiveInfo(){
        std::cout << "Joint Data: \n";
        std::cout << "pos: " << joint_pos_.transpose()*57.3 << std::endl;
        // std::cout << "vel: " << joint_vel_.transpose() << std::endl;
        // std::cout << "tau: " << joint_tau_.transpose() << std::endl;
        std::cout << "Imu Data: \n";
        std::cout << "rpy: " << rpy_.transpose() << std::endl;
        std::cout << "acc: " << acc_.transpose() << std::endl;
        std::cout << "omg: " << omg_.transpose() << std::endl;
    }

    void DisplayAxisValue(){
        // UserCommand* uc_cmd = uc_ptr_->GetUserCommand();
        auto cmd = uc_ptr_->GetUserCommand();
        std::cout << "User Command Input: \n";
        std::cout << "axis value:  " << cmd->forward_vel_scale << " " 
                                     << cmd->side_vel_scale << " "
                                     << cmd->turnning_vel_scale << std::endl;
        std::cout << "target mode: " << int(cmd->target_mode) << std::endl;
    }


public:
    IdleState(const RobotName& robot_name, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_name, state_name, data_ptr){
        }
    ~IdleState(){}

    virtual void OnEnter() {
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::WaitingForStand);
        enter_state_time_ = ri_ptr_->GetInterfaceTimeStamp();
    };
    virtual void OnExit() {
        first_enter_flag_ = false;
    }
    virtual void Run() {
        GetProprioceptiveData();
        joint_normal_flag_ = JointDataNormalCheck();
        imu_normal_flag_ = ImuDataNormalCheck();
        // std::cout << ri_ptr_->GetInterfaceTimeStamp() << " | " << enter_state_time_ << " " << first_enter_flag_ << std::endl;
        cnt++;

        // if(first_enter_flag_ && ri_ptr_->GetInterfaceTimeStamp() - enter_state_time_ > 2.){//to confirm right state input
        //     DisplayProprioceptiveInfo();
        //     DisplayAxisValue();
        // }
        MatXf cmd = MatXf::Zero(cp_ptr_->dof_num, 5);
        // cmd(6,4)=1.2;
        if(cnt%4000==0){
        DisplayProprioceptiveInfo();
        }
        // std::cout<<"cmd:"<<cmd.transpose()<<std::endl;

        ri_ptr_->SetJointCommand(cmd);
    }
    virtual bool LoseControlJudge() {
        return false;
    }
    virtual StateName GetNextStateName() {
        if(!joint_normal_flag_ || !imu_normal_flag_ && false) {
            std::cout << "joint status: " << joint_normal_flag_ << " | imu status: " << imu_normal_flag_ << std::endl;
            // std::cout<< "1 is normal, 0 is abnormal!!"<<std::endl;
            return StateName::kIdle;
        }
        // if(uc_ptr_->GetUserCommand()->safe_control_mode!=0){
        //     std::cout<<"safe control"<<uc_ptr_->GetUserCommand()->safe_control_mode<<std::endl;
        //     return StateName::kIdle;
        // }
        if(uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::StandingUp))
            return StateName::kStandUp;
        return StateName::kIdle;
    }
};

};//namespace humanoid


#endif