/**
 * @file ecan_hardware_interface.hpp
 * @brief hardware interface for ecan
 * @author mazunwang
 * @version 1.0
 * @date 2024-08-19
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef ECAN_HARDWARE_INTERFACE_HPP_
#define ECAN_HARDWARE_INTERFACE_HPP_

#include "common_types.h"
#include "dds_types.h"
#include "robot_interface.h"
#include "drdds/core/drdds_core.h"

struct JointConfig{
    float dir;
    float offset;
};

class EcanHardwareInterface : public RobotInterface{
protected:
    double ri_ts_, imu_leg_ts_, imu_body_ts_;
    Vec3f omega_body_, rpy_body_, acc_body_;
    Vec3f omega_leg_, rpy_leg_, acc_leg_;
    VecXf joint_pos_, joint_vel_, joint_tau_;
    VecXf motor_temperture_, driver_temperture_;
    float *pos_offset_;
    float *joint_dir_;
    bool *data_updated_;
    JointConfig *joint_config_;
    std::vector<uint16_t> driver_status_;
    std::vector<uint16_t> joint_data_id_;
    BatteryInfo_dds battery_info_[2];
    std::vector<uint16_t> battery_data_;

    double time_stamp_;

    ChannelJointsData* joint_cmd_pub_;  
    ChannelJointsData* joint_data_sub_;
    ChannelImuData* imu_leg_data_sub_;
    ChannelImuData* imu_body_data_sub_;
    ChannelExeHealth* health_data_sub_;

    bool IsDataUpdatedFinished(){
        bool res = data_updated_[0];
        for(int i=0; i < dof_num_; ++i){
            res = res & data_updated_[i];
        }
        return res;
    }

    void ResetPositionOffset(){
        memset(data_updated_, 0, dof_num_*sizeof(bool));
        this->SetJointCommand(MatXf::Zero(dof_num_, 5));
        usleep(100*1000);
        VecXf last_joint_pos = this->GetJointPosition();
        VecXf current_joint_pos = this->GetJointPosition();
        int cnt = 0;
        while (!IsDataUpdatedFinished())
        {
            ++cnt;
            this->RefreshRobotData();
            current_joint_pos = this->GetJointPosition();
            
            for(int i=0;i<dof_num_;++i){
                if(!data_updated_[i] && current_joint_pos(i) != last_joint_pos(i)){
                    data_updated_[i] = true;
                    std::cout << "joint " << i << " data updated at " << cnt << " cnt!" << std::endl;
                }
            }
            last_joint_pos = current_joint_pos;
            usleep(1000);
            if(cnt > 10000){
                std::cout << "joint data update is not finished\n";
            }
        }
        for(int i=1;i<dof_num_;i+=4){
            if(current_joint_pos(i) < -210. / 180. * M_PI){
                pos_offset_[i] = pos_offset_[i] + 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            }else if(current_joint_pos(i) > 90. / 180. * M_PI){
                pos_offset_[i] = pos_offset_[i] - 360.;
                std::cout << "joint " << i << " offset is changed to " << pos_offset_[i] << "\n";
            }
        }
        for(int i=0;i<dof_num_;++i){
            joint_config_[i].dir = joint_dir_[i];
            joint_config_[i].offset = Deg2Rad(pos_offset_[i]);
        }
    }

public:
    int run_cnt_ =0;    
    EcanHardwareInterface(const std::string& robot_name, const int &dof_num):RobotInterface(robot_name, dof_num){
        std::cout << robot_name << " is using Ecan Hardware Interface" << std::endl;
        joint_pos_ = VecXf::Zero(dof_num_);
        joint_vel_ = VecXf::Zero(dof_num_);
        joint_tau_ = VecXf::Zero(dof_num_);
        motor_temperture_ = VecXf::Zero(dof_num_);
        driver_temperture_ = VecXf::Zero(dof_num_);

        pos_offset_ = new float[dof_num_];
        joint_dir_ = new float[dof_num_];
        data_updated_ = new bool[dof_num_];
        joint_config_ = new JointConfig[dof_num_];

        driver_status_.resize(dof_num_);
        joint_data_id_.resize(dof_num_);
        battery_data_.resize(BATTERY_DATA_SIZE);

        joint_cmd_pub_ = new ChannelJointsData("/JOINTS_CMD", 0);
        joint_data_sub_ = new ChannelJointsData(std::bind(&EcanHardwareInterface::Handler, this, std::placeholders::_1), "/JOINTS_DATA", 0);
        imu_leg_data_sub_ = new ChannelImuData(std::bind(&EcanHardwareInterface::HandlerIMULeg, this, std::placeholders::_1), "/IMU_DATA_LEG", 0);
        imu_body_data_sub_ = new ChannelImuData(std::bind(&EcanHardwareInterface::HandlerIMUBody, this, std::placeholders::_1), "/IMU_DATA_BODY", 0);
        health_data_sub_ = new ChannelExeHealth(std::bind(&EcanHardwareInterface::HandlerHealth, this, std::placeholders::_1), "/BATTERY_DATA", 0);

        sleep(1);
        joint_cmd_pub_->data_->data().joints_data().resize(dof_num_);

        uint16_t control_word[] = {kIndexDisable, kIndexErrorReset, kIndexEnable};
        // uint16_t control_word[] = {kIndexDisable, kIndexDisable, kIndexDisable};

        for(int j=0;j<3;++j){
            for(int i=0;i<dof_num_;++i){
                joint_cmd_pub_->data_->data().joints_data()[i].position() = 0;
                joint_cmd_pub_->data_->data().joints_data()[i].velocity() = 0;
                joint_cmd_pub_->data_->data().joints_data()[i].torque() = 0;
                joint_cmd_pub_->data_->data().joints_data()[i].motion_temp_kp().kp(0);
                joint_cmd_pub_->data_->data().joints_data()[i].driver_temp_kd().kd(0);
                joint_cmd_pub_->data_->data().joints_data()[i].control_word(control_word[j]);
            }
            joint_cmd_pub_->Write();
            usleep(10*1000);
        }
    }
    virtual ~EcanHardwareInterface(){
        delete [] pos_offset_;
        delete [] joint_dir_;
        delete [] data_updated_;
        delete [] joint_config_;
    }

    virtual void Start(){
    }

    virtual void Stop(){
    }

    virtual double GetInterfaceTimeStamp(){
        return ri_ts_;
    }
    virtual VecXf GetJointPosition() {
        return joint_pos_;
    }
    virtual VecXf GetJointVelocity() {
        return joint_vel_;
    }
    virtual VecXf GetJointTorque() {
        return joint_tau_;
    }
    virtual Vec3f GetImuBodyRpy() {
        return rpy_body_;
    }
    virtual Vec3f GetImuBodyAcc() {
        return acc_body_;
    }
    virtual Vec3f GetImuBodyOmega() {
        std::cout<<"omega_body_"<<omega_body_<<std::endl;
        return omega_body_;
    }
    virtual Vec3f GetImuRpy() {
        return rpy_leg_;
    }
    virtual Vec3f GetImuAcc() {
        return acc_leg_;
    }
    virtual Vec3f GetImuOmega() {
        return omega_leg_;
    }
    virtual VecXf GetContactForce() {
        return VecXf::Zero(4);
    }
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input){
        // std::cout << "send: \n" << input.transpose() << std::endl;

        for(int i=0;i<dof_num_;++i){
            joint_cmd_pub_->data_->data().joints_data()[i].position() = (input(i, 1) - joint_config_[i].offset)*joint_config_[i].dir;
            joint_cmd_pub_->data_->data().joints_data()[i].velocity() = input(i, 3)*joint_dir_[i];
            joint_cmd_pub_->data_->data().joints_data()[i].torque() = input(i, 4)*joint_dir_[i];
            joint_cmd_pub_->data_->data().joints_data()[i].motion_temp_kp().kp(input(i, 0));
            joint_cmd_pub_->data_->data().joints_data()[i].driver_temp_kd().kd(input(i, 2));
            joint_cmd_pub_->data_->data().joints_data()[i].control_word(kIndexMotorControl);
        }
        joint_cmd_ = input;
        joint_cmd_pub_->Write();
    }
    virtual VecXf GetMotorTemperture(){
        return motor_temperture_;
    }
    virtual VecXf GetDriverTemperture(){
        return driver_temperture_;
    }
    virtual double GetImuTimestamp(){
        return imu_leg_ts_;
    }
    virtual double GetImuBodyTimestamp(){
        return imu_body_ts_;
    }
    virtual std::vector<uint16_t> GetBatteryData(){
        return battery_data_;
    }
    virtual std::vector<uint16_t> GetDriverStatusWord(){
        return driver_status_;
    }
    virtual std::vector<uint16_t> GetJointDataID(){
        return joint_data_id_;
    }
    virtual void RefreshRobotData(){
    }

    void Handler(const drdds::msg::JointsData* data) {
        ++run_cnt_;
        for(int i=0;i<dof_num_;++i){
            joint_pos_(i) = data->data().joints_data().at(i).position()*joint_dir_[i] + pos_offset_[i] / 180. * M_PI;
            joint_vel_(i) = data->data().joints_data().at(i).velocity()*joint_dir_[i];
            joint_tau_(i) = data->data().joints_data().at(i).torque()*joint_dir_[i];
            motor_temperture_(i) = float(data->data().joints_data().at(i).motion_temp_kp().motion_temp());
            driver_temperture_(i) = float(data->data().joints_data().at(i).driver_temp_kd().driver_temp());
            driver_status_[i] = data->data().joints_data().at(i).status_word();
            joint_data_id_[i] = uint16_t(run_cnt_);
        }
        // std::cout<<"joint upload pos:"<< data->data().joints_data().at(5).position()<<std::endl;
        // std::cout<<"pos_offset_:"<<pos_offset_[5]<<std::endl;
        // std::cout<<"final jonit pos:"<<joint_pos_(5)<<std::endl;
        ri_ts_ = GetTimestampMs()/1000.;
    }

    void HandlerIMULeg(const drdds::msg::ImuData* data) {
        rpy_leg_ = Vec3f(Deg2Rad(data->data().roll()), Deg2Rad(data->data().pitch()), Deg2Rad(data->data().yaw()));
        acc_leg_ << data->data().acc_x(), data->data().acc_y(), data->data().acc_z();
        omega_leg_ << data->data().omega_x(), data->data().omega_y(), data->data().omega_z();
        imu_leg_ts_ = GetTimestampMs();
    }

    void HandlerIMUBody(const drdds::msg::ImuData* data) {
        rpy_body_ = Vec3f(Deg2Rad(data->data().roll()), Deg2Rad(data->data().pitch()), Deg2Rad(data->data().yaw()));
        acc_body_ << data->data().acc_x(), data->data().acc_y(), data->data().acc_z();
        omega_body_ << data->data().omega_x(), data->data().omega_y(), data->data().omega_z();
        imu_body_ts_ = GetTimestampMs();
        // std::cout<<"acc_body_:"<<acc_body_.transpose()<<std::endl;
    }

    void HandlerHealth(const drdds::msg::ExeHealth* data) {
        battery_data_[0] = uint8_t(data->data().battery_info().at(1).battery_level());        // 电量
        battery_data_[1] = uint16_t(data->data().battery_info().at(1).voltage());            //电压 
        battery_data_[2] = uint16_t(data->data().battery_info().at(1).current());            //电流//现在电池上报有点问题 只有正值
        battery_data_[3] = uint16_t(data->data().battery_info().at(1).protected_state()); //故障报警
        // std::cout<<"battery_data:"<<battery_data_[0]<<", "<<battery_data_[1]<<", "<<battery_data_[2]<<", "<<battery_data_[3]<<", "<<std::endl;
    }
};





#endif