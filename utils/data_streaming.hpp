/**
 * @file data_streaming.hpp
 * @brief 
 * @author mazunwang
 * @version 1.0
 * @date 2024-04-28
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef DATA_STREAMING_HPP_
#define DATA_STREAMING_HPP_

#include "json.hpp"
#include "robot_interface.h"
#include "user_command_interface.h"
#include "basic_function.hpp"

#define LOCAL_PORT 10121

class DataStreaming{
private:
    int sockfd_;
    sockaddr_in remote_addr_;
    const int remote_port_;
    const std::string remote_ip_;

    const int record_freq_ = 1; //5;

    std::thread record_thread_;

    FILE *fp_;
    std::string file_name_;
    std::ofstream file_;
    std::vector<float> scope_data_;
    
    nlohmann::json data_json_;
    int run_cnt_ = 0;
    const bool enable_online_plot_;
    const bool enable_data_record_;
    float time_stamp_;

    std::shared_ptr<interface::RobotInterface> ri_ptr_;
    std::shared_ptr<interface::UserCommandInterface> uc_ptr_;

    void GetDataStreaming(){
        if(!ri_ptr_) return;
        VecXf pos = ri_ptr_->GetJointPosition();
        VecXf vel = ri_ptr_->GetJointVelocity();
        VecXf tau = ri_ptr_->GetJointTorque();
        VecXf temp = ri_ptr_->GetMotorTemperture();
        VecXf d_temp = ri_ptr_->GetDriverTemperture();
        Vec3f rpy = ri_ptr_->GetImuRpy();
        Vec3f acc = ri_ptr_->GetImuAcc();
        Vec3f omg = ri_ptr_->GetImuOmega();
        MatXf jc = ri_ptr_->GetJointCommand();

        this->InsertInterfaceTime(ri_ptr_->GetInterfaceTimeStamp());
        this->InsertJointData("q", pos);
        this->InsertJointData("dq", vel);
        this->InsertJointData("tau", tau);
        // 2024.12.5增加关节下发力矩值
        VecXf tau_cmd = jc.col(0).cwiseProduct(jc.col(1) - pos) + jc.col(2).cwiseProduct(jc.col(3) - vel) + jc.col(4);
        this->InsertJointData("tau_cmd", tau_cmd);
        // 2024.11.25增加电机温度
        this->InsertJointData("motor_emperture", temp);
        this->InsertJointData("driver_emperture", d_temp);
        // 2025.2.26增加关节位置指令、速度、前馈力矩指令
        this->InsertJointData("q_cmd", jc.col(1));
        this->InsertJointData("dq_cmd", jc.col(3));
        this->InsertJointData("tau_ff_cmd", jc.col(4));
        

        // this->InsertJointData("q_cmd", jc.col(1));
        // this->InsertJointData("kd", jc.col(2));
        // this->InsertJointData("dq_cmd", jc.col(3));
        // this->InsertJointData("tau_ff", jc.col(4));

        this->InsertImuData("rpy", rpy);
        this->InsertImuData("acc", acc);
        this->InsertImuData("omg", omg);

        if(!uc_ptr_) return;
        this->InsertCommandData("target_mode", float(int(uc_ptr_->GetUserCommand()->target_mode)));
        // this->InsertStateData("current_state", float(int(StateBase::msfb_.current_state)));
        // this->SendData();
    }


public:
    DataStreaming(bool enable_online_plot=false, bool enable_data_record=false, const std::string& ip="192.168.1.165", int port=9870)://虚拟机无线ip172.16.11.84，有线ip192.168.1.165
    enable_online_plot_(enable_online_plot),
    enable_data_record_(enable_data_record),
    remote_port_(port),
    remote_ip_(ip){
        for(int i=0;i<100;++i){//初始化，原因未知，貌似只在3588上出现，ubuntu版本问题或者arm问题
            this->InsertImuData("rpy", Vec3f::Zero());
            this->InsertImuData("acc", Vec3f::Zero());
            this->InsertImuData("omg", Vec3f::Zero());
            // this->InsertStateData("current_state", 0.f);
        }
    }
    ~DataStreaming(){}

    void SetRobotDataSource(std::shared_ptr<interface::RobotInterface> ri){
        ri_ptr_ = ri;
    }
    void SetUserCommandDataSource(std::shared_ptr<interface::UserCommandInterface> uc){
        uc_ptr_ = uc;
    }

    void Start(){
        if(enable_online_plot_){
            std::cout << "You can plot data online on plotjuggler" << std::endl;
            sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
            memset(&remote_addr_, 0, sizeof(remote_addr_));
            remote_addr_.sin_family = AF_INET;
            remote_addr_.sin_port = htons(remote_port_);
            remote_addr_.sin_addr.s_addr = inet_addr(remote_ip_.c_str());

            // 设置本地地址和端口号
            struct sockaddr_in local_addr;
            std::memset(&local_addr, 0, sizeof(local_addr));
            local_addr.sin_family = AF_INET;
            local_addr.sin_addr.s_addr = htonl(INADDR_ANY); // 绑定到所有网络接口
            local_addr.sin_port = htons(LOCAL_PORT); // 设置本地端口号

            // 绑定 socket 到本地地址和端口号
            if (bind(sockfd_, (struct sockaddr*)&local_addr, sizeof(local_addr)) == -1) {
                std::cerr << "Failed to bind socket" << std::endl;
            }
            if (sockfd_ < 0) {
                perror("robot sender socket creation failed");
            }
        }

        if(enable_data_record_){
            std::time_t now = std::time(nullptr);
            std::tm* localTime = std::localtime(&now);
            char buffer[100];
            std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", localTime);
            file_name_ = "../data/";
            
            file_name_.append(std::string(buffer));
            file_name_.append(".csv");

            std::cout << "data is recorded in " << file_name_ << std::endl;
            file_.open(file_name_);
            file_.close();

            char username[256];
            if (getlogin_r(username, sizeof(username)) == 0) {
                std::cout << "Username: " << username << std::endl;
            } else {
                std::cerr << "Failed to get username" << std::endl;
            }

            std::string command = "chown " + std::string(username) + ":" + std::string(username) + " " + file_name_;
            int result = system(command.c_str());
            if (result == 0) {
                std::cout << "File permissions changed to 0666 (read and write for all users)" << std::endl;
            } else {
                std::cerr << "Failed to change file permissions" << std::endl;
            }

            file_.open(file_name_);
            if (!file_.is_open()) {
                std::cerr << "Failed to open file " << file_name_ << std::endl;
            }
        }
        scope_data_.resize(10, 0);        

        time_stamp_ = 0.f;
        record_thread_ = std::thread(std::bind(&DataStreaming::SendData, this));
    }

    void Stop(){
        record_thread_.join();
    }

    void SendData(){

        while(true){
            if(data_json_["t_i"]!=time_stamp_){
                ++run_cnt_;
                // std::cout << "run_cnt: " <<run_cnt_ << std::endl;
                if(run_cnt_ == 1&&enable_data_record_) WriteItemNameToCsv();
            }
            if(run_cnt_%record_freq_==0){
                this->GetDataStreaming();
                float ts = GetTimestampMs();
                data_json_["t_r"] = ts/1000.;
                data_json_["scope"] = scope_data_;
                if(enable_data_record_){
                    WriteDataToCsv();
                }
                if(enable_online_plot_){
                    std::string json_str = data_json_.dump();
                    int nbytes = sendto(sockfd_, json_str.c_str(), json_str.length(), 0,
                                (struct sockaddr*)&remote_addr_, sizeof(remote_addr_));
                    if(nbytes < 0){
                        std::cerr << "Error: Could not send message" << std::endl;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
    }

    void InsertInterfaceTime(float data){
        data_json_["t_i"] = data;
    }

    void InsertJointData(const std::string &name, float data){
        data_json_["joint"][name] = data;
    }
        
    void InsertJointData(const std::string &name, const VecXf& data){
        std::vector<float> vec(data.data(), data.data()+data.size());
        data_json_["joint"][name] = vec;
    }    

    void InsertImuData(const std::string &name, float data){
        data_json_["imu"][name] = data;
    }

    void InsertImuData(const std::string &name, const Vec3f& data){
        std::vector<float> vec(data.data(), data.data()+data.size());
        data_json_["imu"][name] = vec;
    }

    void InsertCommandData(const std::string &name, float data){
        data_json_["usr_cmd"][name] = data;
    }

    // void InsertStateData(const std::string &name, float data){
    //     data_json_["state"][name] = data;
    // }

    void InsertScopeData(int index, float data){
        scope_data_[index] = data;
    }

    void WriteItemNameToCsv(){
        file_ << "run_cnt" << "," << "t_r" << "," << "t_i" << ",";
        // for(int i=0;i<3;++i){
        //     file_ << "rpy"+std::to_string(i) << ",";
        // }
        // for(int i=0;i<3;++i){
        //     file_ << "acc"+std::to_string(i) << ",";
        // }
        // for(int i=0;i<3;++i){
        //     file_ << "omg"+std::to_string(i) << ",";
        // }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            file_ << "q"+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            file_ << "dq"+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            file_ << "tau"+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            file_ << "tau_cmd"+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            file_ << "temp"+std::to_string(i) << ",";
        }
        for(int i=0;i<ri_ptr_->dof_num_;++i){
            file_ << "d_temp"+std::to_string(i) << ",";
        }

        // for(int i=0;i<ri_ptr_->dof_num_;++i){
        //     file_ << "q_cmd"+std::to_string(i) << ",";
        // }
        // for(int i=0;i<ri_ptr_->dof_num_;++i){
        //     file_ << "tau_ff"+std::to_string(i) << ",";
        // }
        // file_ << "state" << ",";
        for(int i=0;i<scope_data_.size();++i){
            file_ << "scope"+std::to_string(i) << ",";
        }
        file_ << "\n";
    }
    

    void WriteDataToCsv(){
        std::ostringstream oss;
        // oss << std::fixed << std::setprecision(3);
        oss << run_cnt_ << "," << data_json_["t_r"] << "," << data_json_["t_i"] << ",";
        // for(int i=0;i<data_json_["imu"].at("rpy").size();++i){
        //     oss << data_json_["imu"].at("rpy")[i] << ",";
        // }
        // for(int i=0;i<data_json_["imu"].at("acc").size();++i){
        //     oss << data_json_["imu"].at("acc")[i] << ",";
        // }
        // for(int i=0;i<data_json_["imu"].at("omg").size();++i){
        //     oss << data_json_["imu"].at("omg")[i] << ",";
        // }
        for(int i=0;i<data_json_["joint"].at("q").size();++i){
            oss << data_json_["joint"].at("q")[i] << ",";
        }
        for(int i=0;i<data_json_["joint"].at("dq").size();++i){
            oss << data_json_["joint"].at("dq")[i] << ",";
        }
        for(int i=0;i<data_json_["joint"].at("tau").size();++i){
            oss << data_json_["joint"].at("tau")[i] << ",";
        }
        for(int i=0;i<data_json_["joint"].at("tau_cmd").size();++i){
            oss << data_json_["joint"].at("tau_cmd")[i] << ",";
        }
        for(int i=0;i<data_json_["joint"].at("temp").size();++i){
            oss << data_json_["joint"].at("temp")[i] << ",";
        }
        for(int i=0;i<data_json_["joint"].at("d_temp").size();++i){
            oss << data_json_["joint"].at("d_temp")[i] << ",";
        }
        // for(int i=0;i<data_json_["joint"].at("q_cmd").size();++i){
        //     oss << data_json_["joint"].at("q_cmd")[i] << ",";
        // }
        // for(int i=0;i<data_json_["joint"].at("tau_ff").size();++i){
        //     oss << data_json_["joint"].at("tau_ff")[i] << ",";
        // }
        // oss << data_json_["state"].at("current_state") << ",";
        for(int i=0;i<scope_data_.size();++i){
            oss << scope_data_[i] << ",";
        }
        oss << "\n";
        file_ << oss.str();
    }
};

#endif