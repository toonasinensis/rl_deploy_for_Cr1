/**
 * @file rl_control_state.hpp
 * @brief rl policy runnning state
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef HUMANOID_RL_CONTROL_STATE_HPP_
#define HUMANOID_RL_CONTROL_STATE_HPP_

#include "state_base.h"
#include "policy_runner_base.hpp"
// #include "humanoid_wbc_policy_runner.hpp"
#include "cr1pro_wbc_policy_runner.hpp"
namespace humanoid{
class RLControlState : public StateBase
{
private:
    RobotBasicState rbs_;
    int state_run_cnt_;

    std::shared_ptr<PolicyRunnerBase> policy_ptr_;
    std::shared_ptr<CR1PROWBCPolicyRunner> humanoid_wbc_policy_;
    std::thread run_policy_thread_;
    bool start_flag_ = true;

    float policy_cost_time_ = 1;

    void UpdateRobotObservation(){
        rbs_.base_rpy     = ri_ptr_->GetImuRpy();
        rbs_.base_rot_mat = RpyToRm(rbs_.base_rpy);
        rbs_.base_quat = RpyToQuat(rbs_.base_rpy);
        rbs_.base_omega   = ri_ptr_->GetImuOmega();
        rbs_.base_acc     = ri_ptr_->GetImuAcc();
        rbs_.joint_pos    = ri_ptr_->GetJointPosition();
        rbs_.joint_vel    = ri_ptr_->GetJointVelocity();
        rbs_.joint_tau    = ri_ptr_->GetJointTorque();
    }

    void PolicyRunner(){
        int run_cnt_record = -1;
 

        while (start_flag_){
            // std::cout<<"policy_ptr_->decimation_ "<<policy_ptr_->decimation_ <<std::endl;
            if(state_run_cnt_%policy_ptr_->decimation_ == 0 && state_run_cnt_ != run_cnt_record){
                timespec start_timestamp, end_timestamp;
                clock_gettime(CLOCK_MONOTONIC,&start_timestamp);
 
                auto ra = policy_ptr_->GetRobotAction(rbs_, *(uc_ptr_->GetUserCommand()));

                VecXf joint_pos_lower(cp_ptr_->dof_num_), joint_pos_upper(cp_ptr_->dof_num_);
                auto left_arm_joint_lower = cp_ptr_->arm_joint_lower_;
                auto right_arm_joint_lower = cp_ptr_->arm_joint_lower_;//这合理吗??????????

                auto left_arm_joint_upper = cp_ptr_->arm_joint_upper_;
                auto right_arm_joint_upper = cp_ptr_->arm_joint_upper_;//这合理吗??????????

                right_arm_joint_lower(1) = -cp_ptr_->arm_joint_upper_(1);//1right_shoulder_x_joint
                right_arm_joint_upper(1) = -cp_ptr_->arm_joint_lower_(1);//1right_shoulder_x_joint镜像

         
                auto left_leg_joint_upper = cp_ptr_->leg_joint_upper_;
                auto right_leg_joint_upper = cp_ptr_->leg_joint_upper_;//这合理吗??????????

                auto left_leg_joint_lower = cp_ptr_->leg_joint_lower_;
                auto right_leg_joint_lower = cp_ptr_->leg_joint_lower_;//这合理吗??????????

                right_leg_joint_lower(1) =- cp_ptr_->leg_joint_upper_(1);//1right_hip_x_joint
                right_leg_joint_upper(1) = -cp_ptr_->leg_joint_lower_(1);//5right_ankle_x_joint镜像

                right_leg_joint_lower(2) =- cp_ptr_->leg_joint_upper_(2);//2right_hip_z_joint
                right_leg_joint_upper(2) = -cp_ptr_->leg_joint_lower_(2);//5right_ankle_x_joint镜像

                right_leg_joint_lower(5) = -cp_ptr_->leg_joint_upper_(5);//5right_ankle_x_joint镜像
                right_leg_joint_upper(5) = -cp_ptr_->leg_joint_lower_(5);//5right_ankle_x_joint镜像


                joint_pos_lower <<cp_ptr_->waist_joint_lower_,left_arm_joint_lower,right_arm_joint_lower,  \
                left_leg_joint_lower,left_leg_joint_lower,cp_ptr_->neck_joint_lower_;

                joint_pos_upper <<cp_ptr_->waist_joint_upper_,left_arm_joint_upper,right_arm_joint_upper,  \
                left_leg_joint_upper,left_leg_joint_upper,cp_ptr_->neck_joint_upper_;
                 
                //限幅是为了安全起见，实际策略执行时可以去掉
                for(int i=0;i<12;++i){
                    ra.goal_joint_pos(i) = LimitNumber(ra.goal_joint_pos(i), joint_pos_lower(i), joint_pos_upper(i));
                }

                MatXf res = ra.ConvertToMat();
                
                ri_ptr_->SetJointCommand(res);
                run_cnt_record = state_run_cnt_;
                clock_gettime(CLOCK_MONOTONIC,&end_timestamp);
                policy_cost_time_ = (end_timestamp.tv_sec-start_timestamp.tv_sec)*1e3 
                                    +(end_timestamp.tv_nsec-start_timestamp.tv_nsec)/1e6;
                // std::cout << "cost_time:  " << policy_cost_time_ << " ms\n";
                // std::cout << "state_run_cnt_:  " << state_run_cnt_ <<std::endl;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

public:
    RLControlState(const RobotName& robot_name, const std::string& state_name, 
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_name, state_name, data_ptr){

 
            std::memset(&rbs_, 0, sizeof(rbs_));
            humanoid_wbc_policy_ = std::make_shared<CR1PROWBCPolicyRunner>("common", cp_ptr_->common_policy_path_, 
                                                                                    cp_ptr_->joint_kp, 
                                                                                    cp_ptr_->joint_kd);
            humanoid_wbc_policy_->SetDefaultJointPos(cp_ptr_->default_joint_pos);
            humanoid_wbc_policy_->DisplayPolicyInfo();
            
            policy_ptr_ = humanoid_wbc_policy_;
            if(!policy_ptr_){
                std::cerr << "error policy" << std::endl;
                exit(0);
            }
            policy_ptr_->DisplayPolicyInfo();
        }
    ~RLControlState(){}

    virtual void OnEnter() {
 
        state_run_cnt_ = -1;
        start_flag_ = true;
        run_policy_thread_ = std::thread(std::bind(&RLControlState::PolicyRunner, this));
 
        policy_ptr_->OnEnter();
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::RLControlMode);
 
    };

    virtual void OnExit() { 
        start_flag_ = false;
        run_policy_thread_.join();
        state_run_cnt_ = -1;
    }

    virtual void Run() {
        UpdateRobotObservation();
        ds_ptr_->InsertScopeData(8, policy_cost_time_);
        state_run_cnt_++;
    }

    virtual bool LoseControlJudge() {
        if(uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::JointDamping)) return true;
        return PostureUnsafeCheck();
    }

    bool PostureUnsafeCheck(){
        Vec3f rpy = ri_ptr_->GetImuRpy();
        if(rpy(0) > 30./180*M_PI || rpy(1) > 45./180*M_PI){
            std::cout << "posture value: " << 180./M_PI*rpy.transpose() << std::endl;
            return true;
        }
        return false;
    }

    virtual StateName GetNextStateName() {
        if(uc_ptr_->GetUserCommand()->safe_control_mode!=0){
            std::cout << "safe_control_mode:" << uc_ptr_->GetUserCommand()->safe_control_mode << std::endl;
            return StateName::kJointDamping;
        }
        if(uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::JointDamping)) return StateName::kJointDamping;

        if(uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::WaitingForStand)) return StateName::kIdle;
        return StateName::kRLControl;
    }
};
};//namespace humanoid;

#endif