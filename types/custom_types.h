#ifndef CUSTOM_TYPES_H_
#define CUSTOM_TYPES_H_

#include "common_types.h"

namespace types{
    enum RobotName{
        CR1ARM,
        CR1LEG,
        CR1A,
        CR1B,
        CR1Pro,
        CR1Standard,
    };

    enum RobotType{
        HumanoidArm,
        HumanoidLeg,
        HumanoidStandard,
        HumanoidPro,
    };

    inline RobotType GetRobotTypeByName(RobotName robot_name){
        if(int(robot_name) == int(RobotName::CR1ARM)){
            return RobotType::HumanoidArm;
        }else if(int(robot_name) == int(RobotName::CR1LEG)){
            return RobotType::HumanoidLeg;
        }else if(int(robot_name) == int(RobotName::CR1Standard)){
            return RobotType::HumanoidStandard;
        }else if(int(robot_name) == int(RobotName::CR1Pro)){
            return RobotType::HumanoidPro;
        }else{
            return RobotType::HumanoidLeg;
        }
    }


    enum RobotMotionState{
        WaitingForStand = 0,
        StandingUp      = 1,
        JointDamping    = 2,
        StartDamping    = 3,
        LieDown         = 4,
        RLControlMode   = 6,
        MimicReady      = 7,
    };

    enum RobotGaitType{

    };

    enum StateName{
        kInvalid      = -1,
        kIdle         = 0,
        kStandUp      = 1,
        kJointDamping = 2,
        kStartDamping = 3,
        kLieDown      = 4,
        kRLControl    = 6,
        kMimicReady   = 7,
    };

    enum RemoteCommandType{
        kKeyBoard = 0,
        kRetroid,
        kSkydroid,
        kDDS,
    };
    

    inline std::string GetAbsPath(){
        char buffer[PATH_MAX];
        if(getcwd(buffer, sizeof(buffer)) != NULL){
            return std::string(buffer);
        }
        return "";
    }
};

#endif