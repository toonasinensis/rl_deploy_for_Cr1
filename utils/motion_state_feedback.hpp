/**
 * @file motion_state_feedback.hpp
 * @brief motion state feedback 
 * @author mazunwang
 * @version 1.0
 * @date 2024-08-27
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef MOTION_STATE_FEEDBACK_HPP
#define MOTION_STATE_FEEDBACK_HPP
#include "common_types.h"

class MotionStateFeedback
{
private:
    uint8_t current_state_;
    uint8_t current_gait_;
    uint8_t last_state_;
    uint8_t last_gait_;
    uint32_t error_code_;

    Vec3f current_vel_;
    Vec3f cmd_vel_;
    Vec3f max_vel_;

    union{
        struct{
            uint32_t joint_pos_limit : 1;
            uint32_t posture_limit : 1;
            uint32_t reserved : 30;
        }error_code_bit_;
        uint32_t old_error_code_;
    };
public:
    MotionStateFeedback(/* args */){
        memset(this, 0, sizeof(MotionStateFeedback));
        std::cout << "Motion State Feedback" << std::endl;
    }
    ~MotionStateFeedback(){

    }

    void UpdateCurrentError(uint32_t error_code){
        error_code_ = error_code;
    }

    void UpdateCurrentState(int state){
        if(uint8_t(state) != current_state_){
            last_state_ = current_state_;
            current_state_ = uint8_t(state);
        }
    }

    void UpdateCurrentGait(uint8_t gait){
        if(gait != current_gait_){
            last_gait_ = current_gait_;
            current_gait_ = gait;
        }
    }

    void UpdateCurrentVel(Vec3f vel){
        current_vel_ = vel;
    }

    void ClearMotionError(){error_code_ = 0;}
    void SetJointPosLimitError(){
        error_code_bit_.joint_pos_limit = 1;
    }
    void SetPostureLimitError(){
        error_code_bit_.posture_limit = 1;
    }

    inline uint8_t GetCurrentState(){return current_state_;}
    inline uint8_t GetCurrentGait(){return current_gait_;}
    inline uint8_t GetLastState(){return last_state_;}
    inline uint8_t GetLastGait(){return last_gait_;}
    inline float GetCurrentVelX(){return current_vel_[0];}
    inline float GetCurrentVelY(){return current_vel_[1];}
};

#endif
