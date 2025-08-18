/**
 * @file jueying_common_policy_runner.hpp
 * @brief common policy runner for jueying
 * @author mazunwang
 * @version 1.0
 * @date 2024-06-06
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef HUMANOID_LEG_POLICY_RUNNER_HPP_
#define HUMANOID_LEG_POLICY_RUNNER_HPP_

#include "policy_runner_base.hpp"

class HumanoidLegPolicyRunner : public PolicyRunnerBase
{
private:
    VecXf kp_, kd_;
    VecXf dof_pos_default_;
    Vec3f max_cmd_vel_;

    // std::vector<c10::IValue> obs_vector_{};

    const std::string policy_path_;

    float action_scale_ ;
    float omega_scale_;
    float dof_vel_scale_;

    Vec3f cmd_vel_scale_ = Vec3f(0., 0., 0.);

    int obs_dim_, obs_history_num_, act_dim_;
    int obs_total_dim_;
    VecXf current_observation_;
    VecXf action_, last_action_;

    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_{nullptr};
    Ort::Env env_;
    std::vector<Ort::Value> ort_inputs_;

    const char* input_names_[1] = {"state"}; // must keep the same as model export
    const char* output_names_[1] = {"output"};


public:
    HumanoidLegPolicyRunner(const std::string& policy_name, const std::string& policy_path, const Vec6f& kp, const Vec6f& kd):
    PolicyRunnerBase(policy_name), policy_path_(policy_path)
    {
        SetDecimation(20);
    }
    ~HumanoidLegPolicyRunner(){}

    void DisplayPolicyInfo(){
    }

    void OnEnter(){
        action_.setZero(act_dim_); 
        last_action_.setZero(act_dim_); 

        current_observation_.setZero(obs_dim_);
        run_cnt_ = 0;
        cmd_vel_input_.setZero();
    }

    RobotAction GetRobotAction(const RobotBasicState& ro, const UserCommand& uc){
        
        RobotAction ra;
        // std::cout<<ra.kd<<std::endl;
        return ra;
    }

    void SetDefaultJointPos(const Vec6f& pos){
        dof_pos_default_.setZero(12); 
        for(int i=0;i<12;++i) {
            dof_pos_default_(i) = pos(i%6);
        }
        dof_pos_default_(7) = -pos(1);//hipx
        dof_pos_default_(8) = -pos(2);//hipz
        dof_pos_default_(11) = -pos(5);//anklex
    }
};


#endif