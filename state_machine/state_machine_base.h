/**
 * @file state_machine_base.h
 * @brief state machine base class for different robot state machine control
 * @author mazunwang
 * @version 1.0
 * @date 2024-08-07
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef STATE_MACHINE_BASE_H_
#define STATE_MACHINE_BASE_H_

#include <bitset>
#include "state_base.h"
#include "safe_controller.hpp"
#include "dr_logger.hpp"
// #include "log4cplus_dr/dr_logger.hpp"
#include <chrono>

class TimeTool {
private:
    int tfd;    /**< Timer descriptor.*/
    int efd;    /**< Epoll descriptor.*/
    int fds, ret; /**< Variables used to initialize the timer.*/
    uint64_t value; /**< Variables used to initialize the timer.*/
    struct epoll_event ev, *evptr; /**< Variables used to initialize the timer.*/
    struct itimerspec time_intv;  /**< Variables used to initialize the timer.*/
public:
    timespec system_time; /**< A class for accurately obtaining time.*/
    /**
    * @brief Initialize timer, input cycle(ms).
    * @param Cycle time unit: ms
    */
    void time_init(int ms) {
        tfd = timerfd_create(CLOCK_MONOTONIC, 0);   //创建定时器
        if (tfd == -1) {
            printf("create timer fd fail \r\n");
        }
        time_intv.it_value.tv_sec = 0; //设定2s超时
        time_intv.it_value.tv_nsec = 1000 * 1000 * ms;
        time_intv.it_interval.tv_sec = time_intv.it_value.tv_sec;   //每隔2s超时
        time_intv.it_interval.tv_nsec = time_intv.it_value.tv_nsec;

        printf("timer start ...\n");
        timerfd_settime(tfd, 0, &time_intv, NULL);  //启动定时器

        efd = epoll_create1(0); //创建epoll实例
        if (efd == -1) {
            printf("create epoll fail \r\n");
            close(tfd);
        }

        evptr = (struct epoll_event *) calloc(1, sizeof(struct epoll_event));
        if (evptr == NULL) {
            printf("epoll event calloc fail \r\n");
            close(tfd);
            close(efd);
        }

        ev.data.fd = tfd;
        ev.events = EPOLLIN;    //监听定时器读事件，当定时器超时时，定时器描述符可读。
        epoll_ctl(efd, EPOLL_CTL_ADD, tfd, &ev); //添加到epoll监听队列中
    }

    /**
    * @brief Acquire interrupt signal
    * @return 1:Enter interrupt 0:no
    */
    int time_interrupt() {
        fds = epoll_wait(efd, evptr, 1, -1);    //阻塞监听，直到有事件发生
        if (evptr[0].events & EPOLLIN) {
            ret = read(evptr->data.fd, &value, sizeof(uint64_t));
            if (ret == -1) {
                printf("read return 1 -1, errno :%d \r\n", errno);
                return 1;
            }
        }
        return 0;
    } /**< Acquire interrupt signal.*/

    /**
    * @brief How long has it been
    * @param Initial time
    */
    double get_now_time(double start_time) {
        clock_gettime(1, &system_time);
        return system_time.tv_sec + system_time.tv_nsec / 1e9 - start_time;
    }

    /**
    * @brief Get current time
    */
    double get_start_time() {
        clock_gettime(1, &system_time);
        return system_time.tv_sec + system_time.tv_nsec / 1e9;
    } /**< Get start time.*/
};


class StateMachineBase
{
private:
    /* data */
    DrLogger logger_temp, logger_pos, logger_vel, logger_torq, logger_imu, logger_bat, logger_stat, logger_error;

public:
    StateMachineBase(const RobotType& robot_type):robot_type_(robot_type),
        logger_temp("Temp"), logger_pos("Pos"), logger_vel("Vel"), logger_error("Error"),
        logger_torq("Torq"), logger_imu("Imu"), logger_bat("Bat"), logger_stat("Stat")
    {
        DrLogger::initialize("CR1", "../log/", "/home/ysc/rl_deploy/conf/log.properties");
        // MotionDataReport mdr;
    }
    virtual ~StateMachineBase(){}

    virtual void Start() = 0;

    virtual void Run() {
        int cnt = 0;
        set_timer.time_init(floor(_dt * 1e3 + 1e-8));
        startTime = set_timer.get_start_time();
        while(true){
            
            auto start = std::chrono::high_resolution_clock::now();

            cnt++;
            ri_ptr_->RefreshRobotData();
            // std::cout<<ri_ptr_->GetInterfaceTimeStamp()<<std::endl;
            if (!set_timer.time_interrupt()) {
 
                // auto t2  = std::chrono::high_resolution_clock::now();
                // auto elapsed2 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
                // std::cout<<"elapsed2 "<<elapsed2<<std::endl;
                current_controller_->Run();

                auto t3  = std::chrono::high_resolution_clock::now();

                 // std::cout<<"elapsed3 "<<elapsed3<<std::endl;
                next_state_name_ = current_controller_ -> GetNextStateName();
                auto t4  = std::chrono::high_resolution_clock::now();

                auto elapsed4 = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();
                // std::cout<<"elapsed4 "<<elapsed4<<std::endl;
                // std::cout <<" current_controller_ -> state_name_"<< current_controller_ -> state_name_ << std::endl;    
                // std::cout <<" next_state_name"<< next_state_name_ << std::endl;

                // mdr = uint8_t(next_state_name_);
                
                if(next_state_name_ != current_state_name_){
                    current_controller_ -> OnExit();
                    std::cout << current_controller_ -> state_name_ << " ------------> ";
                    current_controller_ = GetStateControllerPtr(next_state_name_);
                    std::cout << current_controller_ -> state_name_ << std::endl;
                    current_controller_ ->OnEnter();
                    current_state_name_ = next_state_name_; 
                }
                ++run_cnt_;
                auto t5  = std::chrono::high_resolution_clock::now();
                auto elapsed5 = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();
                // std::cout<<"elapsed5 "<<elapsed5<<std::endl;
                ds_ptr_->InsertScopeData(0, StateBase::msfb_.GetCurrentState());
                ds_ptr_->InsertScopeData(1, StateBase::msfb_.GetCurrentGait());
                ds_ptr_->InsertScopeData(2, sc_ptr_->GetErrorCode() );
                ds_ptr_->InsertScopeData(3, uc_ptr_->GetUserCommand()->target_gait);

                auto t6  = std::chrono::high_resolution_clock::now();
                auto elapsed6 = std::chrono::duration_cast<std::chrono::microseconds>(t6 - t5).count();
                // std::cout<<"elapsed6  "<<elapsed6<<std::endl;

            }
           

            // 0.5s
            // if(cnt == 1000){
            if(sc_ptr_->GetErrorCode() > 0){
                // cnt = 0;
                LogRecord();
            }
            auto end = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                // std::cout<<"elapsed all "<<elapsed<<std::endl;
            // 每次控制耗时（可选）
  
        }
    }
    virtual void Stop() = 0;

    // 将Eigen::VectorXf转换成字符串
    template <typename T>
    std::string vectorToString(T vec) {
        std::stringstream ss;
        for (int i = 0; i < vec.size(); ++i) {
            ss << vec[i];
            if (i < vec.size() - 1) {
                ss << ", "; // 在元素之间添加空格
            }
        }
        return ss.str();
    }

    std::string intToBinary(int num) {
        std::bitset<32> bits(num); // 假设 int 是 32 位
        std::string binary = bits.to_string();
        // 去掉前导零
        size_t first_one = binary.find('1');
        if (first_one != std::string::npos) {
            binary = binary.substr(first_one);
        } else {
            binary = "0"; // 如果全是0
        }
        return binary;
    }

    void LogRecord(){
        // info
        logger_pos.info(vectorToString(ri_ptr_->GetJointPosition()));
        logger_vel.info(vectorToString(ri_ptr_->GetJointVelocity()));
        logger_torq.info(vectorToString(ri_ptr_->GetJointTorque()));
        logger_temp.info(vectorToString(ri_ptr_->GetMotorTemperture()));
        logger_imu.info(vectorToString(ri_ptr_->GetImuRpy()));
        logger_bat.info(vectorToString(ri_ptr_->GetBatteryData()));
        logger_stat.info(std::to_string(StateBase::msfb_.GetCurrentState()));
        logger_error.info(intToBinary(sc_ptr_->GetErrorCode()));
    }


    virtual std::shared_ptr<StateBase> GetStateControllerPtr(StateName state_name) = 0;
    const RobotType robot_type_;
    
    // const RobotName robot_name_;
    // const RemoteCommandType remote_cmd_type_;    

    int run_cnt_ = 0;
    double time_record_ = 0;
    
    double _dt = 0.001, startTime;
    TimeTool set_timer{};

    std::shared_ptr<UserCommandInterface> uc_ptr_;
    std::shared_ptr<RobotInterface> ri_ptr_;
    std::shared_ptr<ControlParameters> cp_ptr_;
    std::shared_ptr<DataStreaming> ds_ptr_;
    std::shared_ptr<SafeController> sc_ptr_;

    std::shared_ptr<StateBase> current_controller_;
    StateName current_state_name_, next_state_name_;
};




#endif