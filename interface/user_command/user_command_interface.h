/**
 * @file user_command_interface.h
 * @brief this file is used for robot's user command input
 * @author mazunwang
 * @version 1.0
 * @date 2024-04-11
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef USER_COMMAND_INTERFACE_H_
#define USER_COMMAND_INTERFACE_H_

#include "common_types.h"
#include "custom_types.h"
#include "motion_state_feedback.hpp"

using namespace types;

namespace interface{

class UserCommandInterface{
private:
    /* data */
public:
    UserCommandInterface(RobotName robot_name=CR1LEG){
        robot_name_ = robot_name;
        usr_cmd_ = new UserCommand();
        std::memset(usr_cmd_, 0, sizeof(UserCommand));
    }
    virtual ~UserCommandInterface(){
        delete usr_cmd_;
    }

    /**
     * @brief start the thread to process user command
     */
    virtual void Start() = 0;

    /**
     * @brief stop the thread 
     */
    virtual void Stop() = 0;

    /**
     * @brief return your user command
     * @return UserCommand ptr
     */
    virtual UserCommand* GetUserCommand() = 0; 

    /**
     * @brief set the motion state feedback 
     * @param  msfb         motion state feedback
     */
    virtual void SetMotionStateFeedback(MotionStateFeedback* msfb){
        msfb_ = msfb;
    }

    MotionStateFeedback *msfb_;
    RobotName robot_name_;
    UserCommand *usr_cmd_;
};
};

#endif