#ifndef KEYBOARD_INTERFACE_HPP_
#define KEYBOARD_INTERFACE_HPP_

#include "user_command_interface.h"
#include "custom_types.h"
#include <cstdio>
#include <functional>
#include <termios.h>
#include <string>

#include <cctype>
#include <sstream>
#define AXIS_STEP 0.1

using namespace interface;
using namespace types;

 

bool isNumber(const std::string& str) {
    std::stringstream ss(str);
    double num;
    ss >> num;
    return ss.eof() && !ss.fail();  // 如果整个字符串是有效数字，则返回 true
}


class KeyboardInterface : public UserCommandInterface
{
private:
    bool start_thread_flag_;
    std::thread kb_thread_;
    void ClipNumber(float &num, float low, float up){
        if(low > up) std::cerr << "error clip" << std::endl;
        if(num < low) num = low;
        if(num > up) num = up;
    }

    double GetCurrentTimeStamp(){
        static timespec startup_timestamp;
        timespec now_timestamp;
        if (startup_timestamp.tv_sec + startup_timestamp.tv_nsec == 0) {
            clock_gettime(CLOCK_MONOTONIC,&startup_timestamp);
        }
        clock_gettime(CLOCK_MONOTONIC,&now_timestamp);
        return (now_timestamp.tv_sec-startup_timestamp.tv_sec)*1e3 
            + (now_timestamp.tv_nsec-startup_timestamp.tv_nsec)/1e6;
    }

public:
    KeyboardInterface(RobotName robot_name=RobotName::CR1LEG):UserCommandInterface(robot_name){ 
        std::cout << "Using Keyboard Command Interface" << std::endl;
    }
    ~KeyboardInterface(){
    }

    virtual void Start(){
        std::memset(usr_cmd_, 0, sizeof(UserCommand));
        start_thread_flag_ = true;
        kb_thread_ = std::thread(std::bind(&KeyboardInterface::Run, this));
    }
    virtual void Stop(){
        start_thread_flag_ = false;
    }
    virtual UserCommand* GetUserCommand() override {return usr_cmd_;}


    void Run(){
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        char input;
        double forward_time_record = GetCurrentTimeStamp();
        double side_time_record = GetCurrentTimeStamp();
        double turnning_time_record = GetCurrentTimeStamp();
        std::cout << "Start Keyboard Listening" << std::endl;
        while (start_thread_flag_) {
            // std::cout << "time: " << usr_cmd_->time_stamp << std::endl;
            usr_cmd_->time_stamp = GetCurrentTimeStamp();
            if(read(STDIN_FILENO, &input, 1) != -1){
                double current_time = GetCurrentTimeStamp();
                // std::cout << "input: " << input << std::endl;
                if(input == 'r'){
                    usr_cmd_->target_mode = uint8_t(RobotMotionState::JointDamping);
                }
                // usr_cmd_->down_shift = false;
                // usr_cmd_->up_shift = false;
                switch(msfb_->GetCurrentState()) {
                    case RobotMotionState::WaitingForStand:
                        if(input=='z'){
                            usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
                        }
                    break;
                    case RobotMotionState::StartDamping:
                        if(input=='z'){
                            usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
                        }
                    break;
                    case RobotMotionState::StandingUp:
                        if(input=='x'){
                            usr_cmd_->target_mode = uint8_t(RobotMotionState::WaitingForStand);
                        }
                        if(input=='v'){
                            usr_cmd_->target_mode = uint8_t(RobotMotionState::MimicReady);
                        }
                        // if(input=='/'){
                        //     usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);
                        // }
                        if(input=='l'){
                            usr_cmd_->target_mode = uint8_t(RobotMotionState::LieDown);
                        }
                        if(input=='c'){
                            // std::cout<<"????????"<<std::endl;

                            usr_cmd_->target_mode = uint8_t(RobotMotionState::CalibrateMode);
                            std::cout << "Target mode: " << static_cast<int>(usr_cmd_->target_mode) << std::endl;

                        }
                    break;
                    case RobotMotionState::MimicReady:
                        if(input=='/'){
                            usr_cmd_->target_mode = uint8_t(RobotMotionState::RLControlMode);
                        }
                        if(input=='z'){
                            usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
                        }
                    break;

                    case RobotMotionState::LieDown:
                        if(input=='z'){
                            usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
                        }
                    break;
                    case RobotMotionState::RLControlMode:
                        if(input=='l'){
                            usr_cmd_->target_mode = uint8_t(RobotMotionState::LieDown);
                        }
                        if(input=='w') {
                            usr_cmd_->forward_vel_scale+=AXIS_STEP;
                            forward_time_record = current_time;
                        }  
                        else if(input=='s') {
                            usr_cmd_->forward_vel_scale-=AXIS_STEP;
                            forward_time_record = current_time;
                        }

                        if(input=='a') {
                            usr_cmd_->side_vel_scale+=AXIS_STEP;
                            side_time_record = current_time;
                        }
                        else if(input=='d') {
                            usr_cmd_->side_vel_scale-=AXIS_STEP;
                            side_time_record = current_time;
                        }
                        
                        if(input=='q') {
                            usr_cmd_->turnning_vel_scale+=AXIS_STEP;
                            turnning_time_record = current_time;
                        }
                        else if(input=='e') {
                            usr_cmd_->turnning_vel_scale-=AXIS_STEP;
                            turnning_time_record = current_time;
                        }
                        
                        if(current_time - forward_time_record > 300.) usr_cmd_->forward_vel_scale = 0;
                        if(current_time - side_time_record > 300.) usr_cmd_->side_vel_scale = 0;
                        if(current_time - turnning_time_record > 300.) usr_cmd_->turnning_vel_scale = 0;

                        ClipNumber(usr_cmd_->forward_vel_scale, -1., 1.);
                        ClipNumber(usr_cmd_->side_vel_scale, -1., 1.);
                        ClipNumber(usr_cmd_->turnning_vel_scale, -1., 1.);
                    break;
                    case CalibrateMode:
                        if(input=='z'){
                            usr_cmd_->target_mode = uint8_t(RobotMotionState::StandingUp);
                        }
                        if(input=='.') {//>
                            usr_cmd_->calibrateJoint = 1.0/180 * 3.14;
                            forward_time_record = current_time;
                        }
                        else if(input==',') {//<
                            usr_cmd_->calibrateJoint = -1.0/180 * 3.14;
                            forward_time_record = current_time;
                        }
                        if((input==']')) {
                            usr_cmd_->target_joint += 1;
                            forward_time_record = current_time;
                        }
                        else if(input=='[') {
                            usr_cmd_->target_joint -= 1;
                            forward_time_record = current_time;
                        }
                        else if(input=='f') {
                            usr_cmd_->finish_calibrate = true;
                            forward_time_record = current_time;
                        }


                    break;
                    default:
                        break;
                }
            }
            // std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

};




#endif