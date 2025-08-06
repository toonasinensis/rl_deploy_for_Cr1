/**
 * @file humanoid_state_machine.hpp
 * @brief for robot to switch control state by user command input
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef HUMANOID_STATE_MACHINE_HPP_
#define HUMANOID_STATE_MACHINE_HPP_

#include "state_base.h"
#include "state_machine_base.h"
#include "humanoid/idle_state.hpp"
#include "humanoid/test_state.hpp"
#include "humanoid/joint_damping_state.hpp"
#include "humanoid/rl_control_state.hpp"

#include "skydroid_gamepad_interface.hpp"
#include "retroid_gamepad_interface.hpp"
#include "keyboard_interface.hpp"

#include "hardware/cr1_hardware_interface.hpp"
#include "hardware/cr1pro_wbc_simlution_interface.hpp"
#include "hardware/cr1pro_wbc_hardware_interface.hpp"
#include "data_streaming.hpp"
#include "safe_controller.hpp"
#include "mimic_ready_state.hpp"
#define SIMULATION_MODE
namespace humanoid{
class HumanoidStateMachine : public StateMachineBase{
private:
    std::shared_ptr<StateBase> idle_controller_;
    std::shared_ptr<StateBase> test_controller_;
    std::shared_ptr<StateBase> joint_damping_controller_;
    std::shared_ptr<StateBase> rl_controller_;
    std::shared_ptr<StateBase> mimic_init_controller_;

    // StateName current_state_name_, next_state_name_;

public:
    const RobotName robot_name_;
    const RemoteCommandType remote_cmd_type_;    

    HumanoidStateMachine(RobotName robot_name, RemoteCommandType rct):StateMachineBase(RobotType::HumanoidLeg),
    robot_name_(robot_name),
    remote_cmd_type_(rct) {
        // robot_name_ = robot_name;
        // remote_cmd_type_ = rct;
    }
    ~HumanoidStateMachine(){}

    void Start(){
        std::string urdf_path = "";
        if(remote_cmd_type_ == RemoteCommandType::kKeyBoard){
            uc_ptr_ = std::make_shared<KeyboardInterface>(robot_name_);
        }else if(remote_cmd_type_ == RemoteCommandType::kRetroid){
            uc_ptr_ = std::make_shared<RetroidGamepadInterface>(12121, robot_name_);
        }else if(remote_cmd_type_ == RemoteCommandType::kSkydroid){
            uc_ptr_ = std::make_shared<SkydroidGamepadInterface>(12121, robot_name_);
        }else{
            std::cerr << "error user command interface! " << std::endl;
            exit(0);
        }
        uc_ptr_->SetMotionStateFeedback(&StateBase::msfb_);

        if(robot_name_ == RobotName::CR1LEG){
            #ifdef  SIMULATION_MODE
                ri_ptr_ = std::make_shared<CR1_PRO_WBC_SimInterface>("CR1LEG", 29);
            #else
                ri_ptr_ = std::make_shared<CR1HardwareInterface>("CR1LEG");
            #endif
            cp_ptr_ = std::make_shared<ControlParameters>(robot_name_);  
        }else if(robot_name_ == RobotName::CR1A){
            ri_ptr_ = std::make_shared<CR1HardwareInterface>("CR1A");
            cp_ptr_ = std::make_shared<ControlParameters>(robot_name_);

        }
        else if(robot_name_ == RobotName::CR1Pro){
            #ifdef  SIMULATION_MODE
                cp_ptr_ = std::make_shared<ControlParameters>(robot_name_);
                ri_ptr_ = std::make_shared<CR1_PRO_WBC_SimInterface>("CR1B",cp_ptr_->dof_num_);        

   
            #else
                ri_ptr_ = std::make_shared<CR1_PRO_WBC_HardwareInterface>("CR1PRO");        
                cp_ptr_ = std::make_shared<ControlParameters>(robot_name_);
   
            #endif
            
        }
        else if(robot_name_ == RobotName::CR1ARM){
            // ri_ptr_ = std::make_shared<CR1ArmHardwareInterface>("CR1ARM");
            // cp_ptr_ = std::make_shared<ControlParameters>(robot_name_);  
        }
        std::shared_ptr<ControllerData> data_ptr = std::make_shared<ControllerData>();
        data_ptr->ri_ptr = ri_ptr_;
        data_ptr->uc_ptr = uc_ptr_;
        data_ptr->cp_ptr = cp_ptr_;
        ds_ptr_ = std::make_shared<DataStreaming>(false, false);
        ds_ptr_->SetRobotDataSource(ri_ptr_);
        ds_ptr_->SetUserCommandDataSource(uc_ptr_);
        data_ptr->ds_ptr = ds_ptr_;

        sc_ptr_ = std::make_shared<SafeController>(HumanoidStandard, "");
        sc_ptr_->SetRobotDataSource(ri_ptr_);
        sc_ptr_->SetUserCommandDataSource(uc_ptr_);

        idle_controller_ = std::make_shared<IdleState>(robot_name_, "idle_state", data_ptr);
        test_controller_ = std::make_shared<TestState>(robot_name_, "test_state", data_ptr);
        mimic_init_controller_ = std::make_shared<MimicReadyState>(robot_name_, "mimic_init_state", data_ptr);
        joint_damping_controller_ = std::make_shared<JointDampingState>(robot_name_, "joint_damping", data_ptr);
        rl_controller_ = std::make_shared<RLControlState>(robot_name_, "rl_control", data_ptr);

        current_controller_ = idle_controller_;
        current_state_name_ = kIdle;
        next_state_name_ = kIdle;

        std::cout << "Controller will be enabled in 3 seconds!!!" << std::endl;
        // std::this_thread::sleep_for(std::chrono::seconds(3)); //for safety 

        ri_ptr_->Start();
        uc_ptr_->Start();
        ds_ptr_->Start();

        #ifndef USE_RAISIM
            sc_ptr_->Start();
        #endif
        
        current_controller_->OnEnter();  
        std::cout<<"start finish"<<std::endl;
    }

    std::shared_ptr<StateBase> GetStateControllerPtr(StateName state_name){
        switch(state_name){
            case StateName::kInvalid:{
                return nullptr;
            }
            case StateName::kIdle:{
                return idle_controller_;
            }
            case StateName::kStandUp:{
                return test_controller_;
            }
            case StateName::kRLControl:{
                return rl_controller_;
            }
            case StateName::kJointDamping:{
                return joint_damping_controller_;
            }
            case StateName::kMimicReady:{
                return mimic_init_controller_;
            }
            default:{
                std::cerr << "error state name" << std::endl;
                return joint_damping_controller_;
            }
        }
        return nullptr;
    }


    void Stop(){
        uc_ptr_->Stop();
        ri_ptr_->Stop();
        ds_ptr_->Stop();
    }

};
};//namespace humanoid
#endif