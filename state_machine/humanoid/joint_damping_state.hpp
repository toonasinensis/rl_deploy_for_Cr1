/**
 * @file joint_damping_state.hpp
 * @brief joint passive control state
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef HUMANOID_JOINT_DAMPING_STATE_HPP_
#define HUMANOID_JOINT_DAMPING_STATE_HPP_

#include "state_base.h"

namespace humanoid{
class JointDampingState : public StateBase{
private:
    float time_record_, run_time_;
    MatXf joint_cmd_;
public:
    JointDampingState(const RobotName& robot_name, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_name, state_name, data_ptr){
            VecXf kd_ = VecXf::Zero(cp_ptr_->dof_num_);
            std::cout<<"kd_.size("<<kd_.size()<<"w"<<cp_ptr_->waist_kd.size() <<"arm "<<cp_ptr_->arm_kd.size()<<cp_ptr_->leg_kd.size()\
            <<" cp_ptr_->neck_kd;"<< cp_ptr_->neck_kd.size()<<std::endl;
            kd_<<cp_ptr_->waist_kd ,cp_ptr_->arm_kd,cp_ptr_->arm_kd, cp_ptr_->leg_kd, cp_ptr_->leg_kd, cp_ptr_->neck_kd;
            joint_cmd_ = MatXf::Zero(cp_ptr_->dof_num_, 5);
            joint_cmd_.col(2) = kd_;
        }
    ~JointDampingState(){}

    virtual void OnEnter() {
        time_record_ = ri_ptr_->GetInterfaceTimeStamp();
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        msfb_.UpdateCurrentState(RobotMotionState::JointDamping);
    };
    virtual void OnExit() {
    }
    virtual void Run() {
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        ri_ptr_->SetJointCommand(joint_cmd_);
    }
    virtual bool LoseControlJudge() {
        return false;
    }
    virtual StateName GetNextStateName() {
        if(run_time_ - time_record_ < 3.){
            return StateName::kJointDamping;
        }
        return StateName::kIdle;
    }
};


};//namespace humanoid;


#endif