#ifndef CR1_HARDWARE_INTERFACE_HPP_
#define CR1_HARDWARE_INTERFACE_HPP_

#include "ecan_hardware_interface.hpp"

class CR1HardwareInterface : public EcanHardwareInterface
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
            // std::cout<<"pos:"<<current_joint_pos.transpose()<<std::endl;
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
        // for(int i=1;i<dof_num_;i+=3){
        //     if(current_joint_pos(i) > Deg2Rad(210)){//肘关节自然垂下为90度，原来的90度作判定容易导致肘关节offset变化导致超限
        //         pos_offset_[i] = pos_offset_[i] - 360.;
        //         std::cout << "joint " << i << " offset is changed\n";
        //     }
        //     if(current_joint_pos(i) < Deg2Rad(-210)){
        //         pos_offset_[i] = pos_offset_[i] + 360.;
        //         std::cout << "joint " << i << " offset is changed\n";
        //     }
        // }
    }

public:
    CR1HardwareInterface(const std::string& robot_name):EcanHardwareInterface(robot_name,31){
        // float init_pos_offset[12] =                             
        //                     {-0., 0., -0., -10., -0., -0.,
        //                      -0., 0., -0., -10., -0., -0.};
        // float joint_dir[12] = {-1, -1, 1, 1, 1, -1,
        //                         1, -1, 1, -1, -1, -1};
        // float init_pos_offset[26] =                             
        //                     {-0., 0., -0., -10., -0., -0.,
        //                      -0., 0., -0., -10., -0., -0.,
        //                      -0., 0., -0., -0., -0., -0., -0.,
        //                      -0., 0., -0., -0., -0., -0., -0.};
        // float joint_dir[26] = {-1, -1, 1, 1, 1, -1,
        //                         1, -1, 1, -1, -1, -1,
        //                         1, -1, 1, 1, 1, 1, 1,
        //                         -1, 1, 1, -1, 1, 1, 1};
        float init_pos_offset[31] = {100., 0., 0.,//腰 wristX Y限位太难标了，标在0，相对准确
                                    90., -25., 170., -44., 170., -90., -90.,//左手
                                    90., 25., -170., -44., -170., -90., 90.,//右手
                                    0*90., -25., -30., -10., 45., 35.,//左腿
                                    0*90., 25., 30., -10., 45., -35.,//右腿
                                    0., 0.};//头
        float joint_dir[31] = { -1, 1, 1,//腰
                                -1, 1, 1, -1, 1, 1, 1,//左手
                                1, 1, 1, -1, 1, 1, 1,//右手
                                1, -1, 1, -1, -1, -1,//左腿
                                -1, 1, 1, -1, -1, -1,//右腿
                                1, 1//头
                                };
        // float init_pos_offset[21] = {100.,//腰 wristX Y限位太难标了，标在0，相对准确
        //                             90., -25., 170., -44.,//左手
        //                             90., 25., -170., -44.,//右手
        //                             0*90., -25., -30., -10., 45., 35.,//左腿
        //                             0*90., 25., 30., -10., 45., -35.//右腿
        //                            };
        // float joint_dir[21] = { -1,//腰
        //                         -1, 1, 1, -1, //左手
        //                         1, 1, 1, -1, //右手
        //                         1, -1, 1, -1, -1, -1,//左腿
        //                         -1, 1, 1, -1, -1, -1,//右腿
        //                         };
        for(int i=0;i<dof_num_;++i){
            pos_offset_[i] = init_pos_offset[i];
            joint_dir_[i] = joint_dir[i];
            data_updated_[i] = false;
            joint_config_[i].dir = joint_dir_[i];
            joint_config_[i].offset = Deg2Rad(pos_offset_[i]);
        }
    }
    ~CR1HardwareInterface(){}

    virtual void Start(){
        time_stamp_ = GetTimestampMs();
        ResetPositionOffset();
    }

    virtual void Stop(){
    }
};





#endif
