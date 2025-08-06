/**
 * @file safe_controller.hpp
 * @brief safe controller to make robot safe
 * @author mazunwang
 * @version 1.0
 * @date 2024-08-15
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

/**
 * 一、关节端
 *      1.关节数据异常（nan值）
 *      2.关节数据不更新
 *      3.关节温度过温
 *      4.驱动器报错
 * 二、imu
 *      1.imu数据异常
 *      2.imu数据不更新
 * 三、电池
 *      1.电池电量过低
 *      2.电池数据不上报？
 *      3.电池温度？
 * 四、手柄
 *      1.手柄断连
 *      
 */

#ifndef SAFE_CONTROLLER_HPP_
#define SAFE_CONTROLLER_HPP_

#include "robot_interface.h"
#include "user_command_interface.h"


union RobotErrorState{ 
    struct{ 
        uint32_t imu_num_error : 1; 
        uint32_t imu_update_overtime: 1;
        uint32_t heatbeat_overtime : 1; 
        uint32_t driver_error: 1; //??? 
        uint32_t motor_heat_warn : 1; 
        uint32_t joint_num_error : 1;
        uint32_t joint_update_error : 1;
        uint32_t battery_low_warn : 1; 
        uint32_t battery_heat_warn : 1;//??? 
        uint32_t cpu_heat_warn : 1;//??? 
        uint32_t cpu_freq_warn : 1;//??? 
        uint32_t driver_heat_warn : 1;//???
        uint32_t reserved : 20; 
    }; 
    uint32_t error_code; 
}; 

class SafeController
{
private:
    RobotType robot_type_;
    std::shared_ptr<interface::RobotInterface> ri_ptr_;
    std::shared_ptr<interface::UserCommandInterface> uc_ptr_;
    std::thread judge_thread_;
    int run_cnt_ = 0; 

    double last_imu_ts_, imu_check_time_;
    bool imu_check_flag_ = true;

    double ri_check_time_, last_ri_ts_;
    VecXf last_joint_pos_, last_joint_vel_, last_joint_tau_;
    std::vector<int> joint_data_same_cnt_;
    bool joint_check_flag_ = true;

    float max_temp_, driver_max_temp_;
    int max_temp_idx_, driver_max_temp_idx_;

    double last_cmd_ts_, cmd_check_time_;
    bool cmd_check_flag_ = true;

    types::UserCommand* usr_cmd_;
    bool start_thread_flag_ = false;

    double driver_error_ts_;
    double joint_data_error_ts_;
    double current_time_;

    RobotErrorState robot_error_state_;
    uint32_t last_error_code_;
    std::map<uint8_t, uint16_t> error_driver_map_;
    uint32_t joint_update_state_;


    bool IsImuDataNormal(){
        double current_imu_ts_ = ri_ptr_->GetImuTimestamp();
        Vec3f rpy = ri_ptr_->GetImuRpy();
        Vec3f acc = ri_ptr_->GetImuAcc();
        Vec3f omg = ri_ptr_->GetImuOmega();
        if(imu_check_flag_){
            last_imu_ts_ = current_imu_ts_;
            imu_check_time_ = current_time_;
            imu_check_flag_ = false;
        }

        bool res = true;
        robot_error_state_.imu_update_overtime = 0;
        if(current_imu_ts_ != last_imu_ts_){
            last_imu_ts_ = current_imu_ts_;
            imu_check_time_ = current_time_;
        }

        if(current_time_ - imu_check_time_ > 0.020){
            robot_error_state_.imu_update_overtime = 1;
            res = false;
            std::cout << (current_time_)<<", " <<(imu_check_time_)<<std::endl;
        }
        for(int i=0;i<3;++i){
            if(std::isnan(rpy(i)) || std::isnan(acc(i)) || std::isnan(omg(i))){
                res = false;
                robot_error_state_.imu_num_error = 1;
            }
        }
        return res;
    }

    bool IsJointDataNormal(){
        return true;
        double ri_ts_ = ri_ptr_->GetInterfaceTimeStamp();// / 1000.;
        VecXf joint_pos = ri_ptr_->GetJointPosition();
        VecXf joint_vel = ri_ptr_->GetJointVelocity();
        VecXf joint_tau = ri_ptr_->GetJointTorque();
        std::vector<uint16_t> joint_id = ri_ptr_->GetJointDataID();
        if(joint_check_flag_){
            joint_check_flag_ = false;
            last_ri_ts_ = ri_ts_;
            last_joint_pos_ = joint_pos;
            last_joint_vel_ = joint_vel;
            last_joint_tau_ = joint_tau;
        }
        // std::cout << "ri_ts: "<< ri_ts_<<std::endl;
        // std::cout <<"check: "<< ri_check_time_/1000.<<std::endl;
        bool res = true;
        robot_error_state_.joint_update_error = 0;
        static bool first_flag = false;
        if(last_ri_ts_ != ri_ts_){
            last_ri_ts_ = ri_ts_;
            ri_check_time_ = current_time_;
            first_flag = true;
        }
        if((current_time_ - ri_check_time_ > 30) && first_flag){
            robot_error_state_.joint_update_error = 1;
            res = false;
        }

        robot_error_state_.joint_num_error = 0;
        joint_update_state_ = 0;
        for(int i=0;i<joint_pos.size();++i){
            if(std::isnan(joint_pos(i)) || std::isnan(joint_vel(i)) || std::isnan(joint_tau(i))){
                robot_error_state_.joint_num_error = 1;
                res = false;
            }
            if(joint_pos(i)==last_joint_pos_(i) && joint_vel(i)==last_joint_vel_(i) && joint_tau(i)==last_joint_tau_(i)){
                joint_data_same_cnt_[i]++;
            }else{
                joint_data_same_cnt_[i] = 0;
            }
            if(joint_data_same_cnt_[i]>5){
                // std::cout << "joint pos: " << joint_pos[i] << std::endl;
                // std::cerr << "joint " << i << " data is not update " << joint_data_same_cnt_[i] << " times at " 
                //                     << ri_ts_ << " | " << current_time_ << " id :" << joint_id[i]  << std::endl;
                // std::cout << "rpy: "<< (ri_ptr_->GetImuRpy()).transpose() <<std::endl;
            }
            if(joint_data_same_cnt_[i]>30){
                std::cerr << "joint " << i << " data is not update" << std::endl;
                robot_error_state_.joint_num_error = 1;
                joint_update_state_ |= (1 << i);
                res = false;
            }
        }
        // ri_check_time_ = ri_ts_;
        last_joint_pos_ = joint_pos;
        last_joint_vel_ = joint_vel;
        last_joint_tau_ = joint_tau;
        return res;
    }

    bool IsDriverStatusNormal(){
        std::vector<uint16_t> status_word = ri_ptr_->GetDriverStatusWord();
        bool res = true;
        robot_error_state_.driver_error = 0;
        for(int i=0;i<status_word.size();++i){
            if(status_word[i]!=1){
                // std::cout << "driver " << i << " status word : " << std::hex << status_word[i] << std::endl;
                error_driver_map_[uint8_t(i)] = status_word[i];
                robot_error_state_.driver_error = 1;
                res = false;
            }
        }
        if(robot_error_state_.driver_error==0){
            error_driver_map_.clear();
        }
        return res;
    }

    bool IsMotorTempertureNormal(){
        VecXf m_t = ri_ptr_->GetMotorTemperture();
        VecXf::Index max_idx, min_idx;
        float max_temp = m_t.maxCoeff(&max_idx);
        float min_temp = m_t.minCoeff(&min_idx);
        max_temp_ = max_temp;
        max_temp_idx_ = max_idx;

        if(run_cnt_ % 5000 == 0){
            std::cout << "Motor Temperture: " << m_t.transpose() << std::endl;
            std::cout << "Max&Min:          " << max_idx << " : " << max_temp << "  |  " << min_idx << " : " << min_temp << std::endl;
        }
        
        // 电机过温保护阈值110度
        if(max_temp > 110 || m_t(3)>70){//左膝关节超过80度高速端磁环退磁
            robot_error_state_.motor_heat_warn = 1;
            return false;
        }else{
            robot_error_state_.motor_heat_warn = 0;
        }

        return true;
    }

    bool IsDriverTempertureNormal(){
        VecXf m_t = ri_ptr_->GetDriverTemperture();
        VecXf::Index max_idx, min_idx;
        float max_temp = m_t.maxCoeff(&max_idx);
        float min_temp = m_t.minCoeff(&min_idx);
        driver_max_temp_ = max_temp;
        driver_max_temp_idx_ = max_idx;
        std::cout << "Driver Temperture: " << m_t.transpose() << std::endl;
        std::cout << "Max&Min:          " << max_idx << " : " << max_temp << "  |  " << min_idx << " : " << min_temp << std::endl;
        // 驱动器过温保护阈值100度
        if(max_temp > 100 || m_t(3)>70){//左膝关节超过80度高速端磁环退磁
            robot_error_state_.driver_heat_warn = 1;
            return false;
        }else{
            robot_error_state_.driver_heat_warn = 0;
        }

        return true;
    }

    bool IsBatteryNormal(){
        std::vector<uint16_t> battery_data = ri_ptr_->GetBatteryData();
        // 电量低于10%
        if(battery_data[0] < 10) {
            // std::cout << "Battery level is " << battery_data[0] << std::endl;
            robot_error_state_.battery_low_warn = 1;
            return false;
        }else{
            robot_error_state_.battery_low_warn = 0;
        }
        return true;
    }

    bool IsUserCommandNormal(){
        double cmd_ts_ = uc_ptr_->GetUserCommand()->time_stamp;
        if(cmd_check_flag_){
            cmd_check_flag_ = false;
            last_cmd_ts_ = cmd_ts_;
            cmd_check_time_ = current_time_;
        }
        robot_error_state_.heatbeat_overtime = 0;
        // 接收命令时间相比上次有变化就会更新
        if(last_cmd_ts_ != cmd_ts_){
            last_cmd_ts_ = cmd_ts_;
            cmd_check_time_ = current_time_;
        }
        // std::cout <<"cmd_ts: "<<cmd_ts_<<std::endl;
        // std::cout <<"delta time : "<< current_time_ - cmd_check_time_ << std::endl;
        // std::cout <<"current: "<< current_time_ << std::endl;
        
        // WiFi断连保护(根据收命令的更新来判断)
        if(current_time_ - cmd_check_time_ > 1000.0) {
            robot_error_state_.heatbeat_overtime = 1;
            // cmd_check_flag_ = true;
            return false;
        }
        return true;
    }


    void PrintRobotErrorState(){
        if(robot_error_state_.imu_num_error){
            std::cout << "imu_num_error: " << ri_ptr_->GetImuRpy().transpose() << " | " 
                                            << ri_ptr_->GetImuAcc().transpose() << " | "
                                            << ri_ptr_->GetImuOmega().transpose() << std::endl;
        }
        if(robot_error_state_.imu_update_overtime){
            std::cout << "imu update overtime: " << last_imu_ts_ << std::endl;
        }

        if(robot_error_state_.joint_num_error){
            std::cout << "joint_data_error : \n" << ri_ptr_->GetJointPosition().transpose() << "\n"
                                                 << ri_ptr_->GetJointVelocity().transpose() << "\n"
                                                 << ri_ptr_->GetJointTorque().transpose() << "\n";


        }
        if(robot_error_state_.joint_update_error){
            std::cout << "joint_update_error : " << std::hex << current_time_ << " " << ri_check_time_ << " "<<current_time_- ri_check_time_<< std::endl;
        }
        if(robot_error_state_.motor_heat_warn){
            std::cout << "motor_over_heat : " << max_temp_idx_ << " | " << max_temp_ << std::endl;
        }
        if(robot_error_state_.driver_heat_warn){
            std::cout << "driver_over_heat : " << driver_max_temp_idx_ << " | " << driver_max_temp_ << std::endl;
        }

        if(robot_error_state_.driver_error){
            std::cout << "driver_error: \n";
            for(const auto& pair : error_driver_map_){
                std::cout << int(pair.first) << " " << std::hex << int(pair.second) << std::endl;
            }
        }

        if(robot_error_state_.battery_low_warn){
            std::cout << "Battery is in low level" << std::endl;
        }
        if(robot_error_state_.battery_heat_warn){

        }

        // if(robot_error_state_.)
    }


public:
    SafeController(RobotType robot_type, const std::string& path):robot_type_(robot_type){
        robot_error_state_.error_code = 0;
        last_error_code_ = 0;
        error_driver_map_.clear();
    }
    ~SafeController(){
    }

    void SetRobotDataSource(std::shared_ptr<interface::RobotInterface> ri){
        ri_ptr_ = ri;
        joint_data_same_cnt_.resize(ri_ptr_->dof_num_, 0);
    }
    void SetUserCommandDataSource(std::shared_ptr<interface::UserCommandInterface> uc){
        uc_ptr_ = uc;
        usr_cmd_ = uc_ptr_->GetUserCommand();
        // usr_cmd_->safe_control_mode = 0;
    }

    void Start(){
        start_thread_flag_ = true;
        judge_thread_ = std::thread(std::bind(&SafeController::Run, this));
    }

    void Run(){
        int tfd;    //定时器描述符
        int efd;    //epoll描述符
        int fds;
        uint64_t value;
        struct epoll_event ev, *evptr;
        struct itimerspec time_intv; //用来存储时间

        tfd = timerfd_create(CLOCK_MONOTONIC, 0);   //创建定时器
        if(tfd == -1)  return ;
        
        time_intv.it_value.tv_sec = 1.; 
        time_intv.it_value.tv_nsec = 1000*1000;
        time_intv.it_interval.tv_sec = 0;   
        time_intv.it_interval.tv_nsec = time_intv.it_value.tv_nsec;

        timerfd_settime(tfd, 0, &time_intv, NULL);  //启动定时器

        efd = epoll_create1(0); //创建epoll实例
        if (efd == -1)  close(tfd);
        
        evptr = (struct epoll_event *)calloc(1, sizeof(struct epoll_event));
        if (evptr == NULL) {
            close(tfd);
            close(efd);
        }

        ev.data.fd = tfd; 
        ev.events = EPOLLIN;    //监听定时器读事件，当定时器超时时，定时器描述符可读。
        epoll_ctl(efd, EPOLL_CTL_ADD, tfd, &ev); //添加到epoll监听队列中
        while(start_thread_flag_){
            fds = epoll_wait(efd, evptr, 1, -1);    //阻塞监听，直到有事件发生
            if(evptr[0].events & EPOLLIN){   
                int ret = read(evptr->data.fd, &value, sizeof(uint64_t));
                if (ret == -1) {
                    continue;
                }
                else{
                }               
            }
            run_cnt_++;
            int safe_control_mode = 0;
            current_time_ = GetTimestampMs();
            if(!IsDriverStatusNormal()){
                usr_cmd_->safe_control_mode = 2; //3;
                driver_error_ts_ = current_time_;
            }
            if(!IsJointDataNormal()){
                usr_cmd_->safe_control_mode = 3;
                joint_data_error_ts_ = current_time_;
            }
            if(run_cnt_%1000==0 && !IsMotorTempertureNormal()){
                usr_cmd_->safe_control_mode = 2;
            }
            // if(run_cnt_%2000==1 && !IsBatteryNormal()){
            //     usr_cmd_->safe_control_mode = 2;
            // }
            // if(!IsUserCommandNormal()){
            //     usr_cmd_->safe_control_mode = 2;
            // } else {
            //     usr_cmd_->safe_control_mode = 0;
            // }
            // if(!IsImuDataNormal()){
            //     usr_cmd_->safe_control_mode = 2;
            // }
            if(last_error_code_ != robot_error_state_.error_code){
                if(robot_error_state_.error_code != 0){
                    std::cout << "error_code: " << std::hex << robot_error_state_.error_code<<std::endl;
                    PrintRobotErrorState();
                } 
                last_error_code_ = robot_error_state_.error_code;
            }
        }
    }

    void Stop(){

    }
    
    inline uint32_t GetErrorCode(){
        return robot_error_state_.error_code;
    }
};





#endif