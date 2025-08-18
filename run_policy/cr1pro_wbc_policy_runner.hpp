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

#ifndef CR1PRO_WBC_POLICY_RUNNER_HPP_
#define CR1PRO_WBC_POLICY_RUNNER_HPP_

#include "policy_runner_base.hpp"
#include "ShmReader.hpp"
#include <iomanip>
#include <chrono>

#include "json_loader.hpp"

class CR1PROWBCPolicyRunner : public PolicyRunnerBase
{
private:
    VecXf kp_, kd_;
    VecXf dof_pos_default_; //dof_pos
    // int cap_num = 22;
    // ShmFloatReader des_joint_read("/shm_joint", 29);  // float 数量为 12
    // ShmFloatReader des_rot_r("/shm_rot", 4);  // float 数量为 12
    // ShmFloatReader des_cp_r("/shm_cp", 22*3);  // float 数量为 12
    // ShmFloatReader des_z_r("/shm_z", 1);  // float 数量为 12

     // std::unique_ptr<ShmFloatReader> des_rot_r;
    // std::unique_ptr<ShmFloatReader> des_cp_r;
    // std::unique_ptr<ShmFloatReader> des_z_r;
    // std::vector<c10::IValue> obs_vector_{};

    const std::string policy_path_;

    int obs_dim_, obs_history_num_, act_dim_;
    int dof_dim = 31;
    int obs_total_dim_;
    VecXf current_observation_;
    VecXf obs_buff;
    VecXf action_, last_action_, action_all_rl, action_all_rbt;//后面两个分别是rl顺序和rbt顺序
    std::vector<int64_t> input_shape_;

    bool use_json_actions = false; // Comment this line to use ONNX inference, uncomment to use JSON actions
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_{nullptr};
    Ort::Env env_;
    std::vector<Ort::Value> ort_inputs_;
    // int obs_dim_ = 0
    const char* input_names_[1] = {"state"}; // must keep the same as model export
    const char* output_names_[1] = {"output"};
    std::string json_path_ = "/home/ubuntu/il/PBHC/eval_log.json";
    JsonLoader loader;
    std::vector<VecXf> joint_data;

    RobotAction ra;
    std::vector<std::string> robot_order = {
    "waist_z_joint", "waist_x_joint", "waist_y_joint",
    "left_shoulder_y_joint", "left_shoulder_x_joint", "left_shoulder_z_joint",
    "left_elbow_joint", "left_wrist_z_joint", "left_wrist_y_joint", "left_wrist_x_joint",
    "right_shoulder_y_joint", "right_shoulder_x_joint", "right_shoulder_z_joint",
    "right_elbow_joint", "right_wrist_z_joint", "right_wrist_y_joint", "right_wrist_x_joint",
    "left_hip_y_joint", "left_hip_x_joint", "left_hip_z_joint",
    "left_knee_joint", "left_ankle_y_joint", "left_ankle_x_joint",
    "right_hip_y_joint", "right_hip_x_joint", "right_hip_z_joint",
    "right_knee_joint", "right_ankle_y_joint", "right_ankle_x_joint","neck1","neck2"};

    std::vector<std::string> policy_order = {
        "left_hip_y_joint", "left_hip_x_joint", "left_hip_z_joint", "left_knee_joint", "left_ankle_y_joint", "left_ankle_x_joint",
        "right_hip_y_joint", "right_hip_x_joint", "right_hip_z_joint", "right_knee_joint", "right_ankle_y_joint", "right_ankle_x_joint",
        "waist_z_joint", "waist_x_joint", "waist_y_joint",
        "left_shoulder_y_joint", "left_shoulder_x_joint", "left_shoulder_z_joint", "left_elbow_joint",
        "right_shoulder_y_joint", "right_shoulder_x_joint", "right_shoulder_z_joint", "right_elbow_joint"
    };

    std::vector<std::string> pd_order = {
        "left_wrist_z_joint", "left_wrist_y_joint", "left_wrist_x_joint",
        "right_wrist_z_joint", "right_wrist_y_joint", "right_wrist_x_joint","neck1","neck2"
    };
    std::vector<std::string> policy_and_pd_order;// = {'left_hip_y_joint', 'right_hip_y_joint', 'waist_z_joint', 'left_hip_x_joint', 'right_hip_x_joint', 'waist_x_joint', 'left_hip_z_joint', 'right_hip_z_joint', 'waist_y_joint', 'left_knee_joint', 'right_knee_joint', 'left_shoulder_y_joint', 'right_shoulder_y_joint', 'left_ankle_y_joint', 'right_ankle_y_joint', 'left_shoulder_x_joint', 'right_shoulder_x_joint', 'left_ankle_x_joint', 'right_ankle_x_joint', 'left_shoulder_z_joint', 'right_shoulder_z_joint', 'left_elbow_joint', 'right_elbow_joint'};    // B
    std::vector<float> values;             // values in A

    std::vector<int> robot2policy_idx;
    std::vector<int> policyandpd2robot_idx;

    //rl 相关参数
    float obs_scales_omega = 0.25;
    float obs_scales_projected_gravity = 1.0;
    float obs_scales_joint_pos = 1.0;
    float obs_scale_joint_vel = 0.05;
    float obs_scale_actions = 1.0;
    float phase_now = 0.0;
    float action_scale = 1.0;

    int data_cnt=0;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time ;


public:
    CR1PROWBCPolicyRunner(const std::string& policy_name, const std::string& policy_path, const VecXf& kp, const VecXf& kd):
    PolicyRunnerBase(policy_name), policy_path_(policy_path),env_(ORT_LOGGING_LEVEL_WARNING, "CR1PROWBCPolicyRunner"),
    session_options_{},
    session_{nullptr},
    memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
    {

        SetDecimation(20);
          // 设置 ONNX SessionOptions
        session_options_.SetIntraOpNumThreads(1);
        session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        if (access(policy_path_.c_str(), F_OK) != 0) {
            std::cerr << "Model file not found: " << policy_path_ << std::endl;
            throw std::runtime_error("Model file missing");
            }

        // 加载模型
        session_ = Ort::Session(env_, policy_path_.c_str(), session_options_);

        obs_dim_ = 76;
        obs_total_dim_ = 380;
        act_dim_ = 23;
        obs_history_num_ = 5;
        assert(obs_dim_ * obs_history_num_ == obs_total_dim_);

        input_shape_ = {1, obs_total_dim_};
        //dummy obs ,test the model is right
        for(int i = 0; i < 10; ++i)
        {
            obs_buff.setZero(obs_total_dim_); // 你这里应该用 obs_total_dim_
            action_ = Onnx_infer( obs_buff );
        }
        
        std::cout<<"model import right!!!!"<<std::endl;

        //init robot action
        obs_buff.setZero(obs_total_dim_);
        

        ra.goal_joint_pos.resize(dof_dim);
        ra.goal_joint_vel.resize(dof_dim);
        ra.kp.resize(dof_dim);
        
        ra.kd.resize(dof_dim);
        ra.tau_ff.resize(dof_dim);

        ra.kp = kp;
        ra.kd = kd;

        //确保变量全部强制设为0，防止因为变长，导致出现未知的数字：
        ra.tau_ff.setZero();
        ra.goal_joint_vel.setZero();

        policy_and_pd_order = policy_order;   //+pd_order;
        policy_and_pd_order.insert(policy_and_pd_order.end(), pd_order.begin(), pd_order.end());

        robot2policy_idx = generate_permutation(robot_order, policy_order);
        policyandpd2robot_idx = generate_permutation(policy_and_pd_order, robot_order);

        
        
        if (use_json_actions) {
            if (!loader.load(json_path_)) {
                std::cerr << "No data file found at " << json_path_ << std::endl;
                throw std::runtime_error("Failed to load JSON file");
            }
            if (!loader.get_key_data("des_joint_pos", joint_data)) {
                std::cerr << "Failed to read key: des_joint_pos" << std::endl;
                throw std::runtime_error("Missing des_joint_pos key in JSON");
            }
            for (const auto& vec : joint_data) {
                if (vec.size() != act_dim_) {
                    std::cerr << "JSON action size mismatch: expected " << act_dim_ << ", got " << vec.size() << std::endl;
                    throw std::runtime_error("Invalid action size in JSON");
                }
            }
            std::cout << "Loaded " << joint_data.size() << " action entries from JSON.\n";
        }
    }
    ~CR1PROWBCPolicyRunner(){}

    std::vector<int> generate_permutation(
        const std::vector<std::string>& from, 
        const std::vector<std::string>& to, 
        int default_index = 0) 
    {
        std::unordered_map<std::string, int> idx_map;
        for (int i = 0; i < from.size(); ++i) {
            idx_map[from[i]] = i;
        }

        std::vector<int> perm;
        for (const auto& name : to) {
            auto it = idx_map.find(name);
            if (it != idx_map.end()) {
                perm.push_back(it->second);
            } else {
                perm.push_back(default_index);  // 如果找不到，就填默认值
            }
        }

        return perm;
    }
    void DisplayPolicyInfo(){
    }

    void OnEnter(){
        action_.setZero(act_dim_); 
        last_action_.setZero(act_dim_); 
        action_all_rl.setZero(dof_dim);
        action_all_rbt.setZero(dof_dim);
        current_observation_.setZero(obs_dim_);
        run_cnt_ = 0;
        cmd_vel_input_.setZero();
        last_time = std::chrono::high_resolution_clock::now();
        data_cnt = 0;
    }

    VecXf Onnx_infer(VecXf current_observation){

        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
            memory_info_,
            current_observation.data(),
            current_observation.size(),
            input_shape_.data(), input_shape_.size()
        );

        std::vector<Ort::Value> inputs;
        inputs.emplace_back(std::move(input_tensor));  // 避免拷贝构造
        
        auto outputs = session_.Run(
            Ort::RunOptions{nullptr},
            input_names_,
            inputs.data(),
            1,
            output_names_,
            1
        );

        float* action_data = outputs[0].GetTensorMutableData<float>();
        Eigen::Map<Eigen::VectorXf> action_map(action_data, act_dim_);
        return VecXf(action_map);  // 返回一个Eigen向量的副本
    }

    RobotAction GetRobotAction(const RobotBasicState& ro, const UserCommand& uc){
        
        data_cnt+=1;
        auto this_time = std::chrono::high_resolution_clock::now();
 
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(this_time - last_time).count();
        last_time = this_time;

        // std::cout << std::fixed << std::setprecision(3) << "Elapsed: " << duration << " ms" << std::endl;

        // below is make cur obs :
        Vec3f base_omgea = ro.base_omega * obs_scales_omega;
        Vec3f gravity = Vec3f(0,0,-1);
        Vec3f projected_gravity = ro.base_rot_mat.inverse() * gravity * obs_scales_projected_gravity;
         
        
        VecXf joint_pos_rl = VecXf(act_dim_);// in rl squenece
        VecXf joint_vel_rl = VecXf(act_dim_);
        VecXf joint_pos_default = VecXf(act_dim_);// in rl squenece

        // std::cout<<"data_cnt"<<data_cnt<<"joint_data.size()"<<joint_data.size()<<std::endl;
        if( data_cnt >= joint_data.size())
        {
            data_cnt = 0;//保持最后一帧数据
            
        }

        phase_now += 0.02/5.06;
        
        if (phase_now > 5.06){phase_now = 0.02;}

        for (int i =0;i<act_dim_;i++)
        {
            joint_pos_rl(i) = ro.joint_pos(robot2policy_idx[i])*obs_scales_joint_pos;
            joint_vel_rl(i) = ro.joint_vel(robot2policy_idx[i])*obs_scale_joint_vel;
            joint_pos_default(i) = dof_pos_default_(robot2policy_idx[i])*obs_scales_joint_pos;
        }
        
        joint_pos_rl -= joint_pos_default;
        // std::cout << "joint_pos_rl" << joint_pos_rl << "joint_pos_default" << joint_pos_default << std::endl;
        current_observation_.setZero(obs_dim_); // 你这里应该用 obs_total_dim_
        VecXf my_vec(23);  // 如果你知道有多少个元素，直接初始化大小
        current_observation_<<base_omgea, projected_gravity, joint_pos_rl, joint_vel_rl, last_action_, phase_now;
        for (size_t i = obs_history_num_ - 1; i > 0; --i) {
            obs_buff.segment(i * obs_dim_, obs_dim_) = obs_buff.segment((i - 1) * obs_dim_, obs_dim_);
        }
        obs_buff.segment(0, obs_dim_) = current_observation_;

        // debug printing observation
        // std::cout << std::fixed << std::setprecision(4);
        // std::cout << "obs_buff:\n";
        // for (int i = 0; i < obs_buff.size(); ++i) {
        //     std::cout << std::setw(12) << obs_buff(i);
        //     if ((i + 1) % 76 == 0 || i == obs_buff.size() - 1) std::cout << std::endl;
        // }

        // load recorded data or use neural network
        if (use_json_actions && !joint_data.empty()) {
            if (data_cnt >= joint_data.size()) {
                data_cnt = 0; // Loop back to start
            }
            action_ = joint_data[data_cnt];
        } else {
            action_ = Onnx_infer(obs_buff);
        }

        // debug printing action
        // std::cout << "action_:\n";
        // for (int i = 0; i < action_.size(); ++i) {
        //     std::cout << std::setw(12) << action_(i);
        //     if ((i + 1) % 76 == 0 || i == action_.size() - 1) std::cout << std::endl;
        // }
        // std::cout << std::endl;

        action_ += dof_pos_default_;
        VecXf no_rl_joint_action = VecXf::Zero(dof_dim - act_dim_);
        action_all_rl << action_, no_rl_joint_action;//把输出为0的，放在后面
        
        // action_all_rl << phase_now, no_rl_joint_action;//假的输入

        last_action_ = action_;
        for(int i = 0;i<dof_dim;i++)
        {
            action_all_rbt(i) = action_all_rl(policyandpd2robot_idx[i]);
         }
        // std::cout<<"dof_pos_default_"<<dof_pos_default_.transpose()<<std::endl;
        
        ra.goal_joint_pos = action_all_rbt  * action_scale;

        ra.goal_joint_vel.setZero();
        auto next_time = std::chrono::high_resolution_clock::now();
        ra.tau_ff(0) = 999;
        auto duration2 = std::chrono::duration_cast<std::chrono::microseconds>(next_time - this_time).count();
            // std::cout << std::fixed << std::setprecision(4)\
            //   << "infer Elapsed: " << duration2 << " us" << std::endl;
        return ra;
    }

    void SetDefaultJointPos(const VecXf& pos){
        dof_pos_default_.setZero(dof_dim); 
        for(int i=0;i<dof_dim;++i) {
            dof_pos_default_(i) = pos(i);
        }
    }
};

#endif