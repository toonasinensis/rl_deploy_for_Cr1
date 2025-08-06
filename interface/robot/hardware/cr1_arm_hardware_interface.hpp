#ifndef CR1_ARM_HARDWARE_INTERFACE_HPP_
#define CR1_ARM_HARDWARE_INTERFACE_HPP_

#include "ecan_hardware_interface.hpp"

class CR1ArmHardwareInterface : public EcanHardwareInterface
{
protected:
    void ResetPositionOffset(){
        memset(data_updated_, 0, dof_num_*sizeof(bool));
        this->SetJointCommand(MatXf::Zero(dof_num_, 5));
        usleep(100*1000);
        VecXf last_joint_pos = this->GetJointPosition();
        VecXf current_joint_pos = this->GetJointPosition();
        int cnt = 0;
        while (!IsDataUpdatedFinished()){
            ++cnt;
            this->RefreshRobotData();
            current_joint_pos = this->GetJointPosition();
            // std::cout<<"pos:"<<current_joint_pos<<std::endl;
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
        for(int i=1;i<dof_num_;i+=3){
            if(current_joint_pos(i) > M_PI / 2.){
                pos_offset_[i] = pos_offset_[i] - 360.;
                std::cout << "joint " << i << " offset is changed\n";
            }
            if(current_joint_pos(i) < Deg2Rad(-210)){
                pos_offset_[i] = pos_offset_[i] + 360.;
                std::cout << "joint " << i << " offset is changed\n";
            }
        }
    }

public:
    CR1ArmHardwareInterface(const std::string& robot_name):EcanHardwareInterface(robot_name,14){
        float init_pos_offset[14] =                             
                            {-0., 0., -0., -0., -0., -0., -0.,
                             -0., 0., -0., -0., -0., -0., -0.,};
        float joint_dir[14] = {1, -1, 1, 1, 1, 1, 1,
                                -1, 1, 1, -1, 1, 1, 1};
        for(int i=0;i<dof_num_;++i){
            pos_offset_[i] = init_pos_offset[i];
            joint_dir_[i] = joint_dir[i];
            data_updated_[i] = false;
            joint_config_[i].dir = joint_dir_[i];
            joint_config_[i].offset = Deg2Rad(pos_offset_[i]);
        }
    }
    ~CR1ArmHardwareInterface(){}

    virtual void Start(){
        time_stamp_ = GetTimestampMs();
        ResetPositionOffset();
    }

    virtual void Stop(){
    }
};





#endif