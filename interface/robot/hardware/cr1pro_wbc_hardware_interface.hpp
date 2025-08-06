#ifndef CR1PRO_WBCHARDWARE_INTERFACE_HPP_
#define CR1PRO_WBCHARDWARE_INTERFACE_HPP_

#include "ecan_hardware_interface.hpp"
#include "json.hpp"   
using json = nlohmann::json;

class CR1_PRO_WBC_HardwareInterface : public EcanHardwareInterface
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
    CR1_PRO_WBC_HardwareInterface(const std::string& robot_name):EcanHardwareInterface(robot_name,31){

         std::ifstream in_file("config/config.json");
        if (!in_file.is_open()) {
            std::cerr << "Failed to open config file" << std::endl;
         }

        json config;
        in_file >> config;

        if (!config.contains("joints")) {
            std::cerr << "Missing joints section" << std::endl;
         }

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
        auto joints = config["joints"];

        std::vector<float> init_pos_offset = load_joint_array(joints["init_pos_offset"]);
        std::vector<float> joint_dir = load_joint_array(joints["joint_dir"]);

        std::cout << "init_pos_offset (" << init_pos_offset.size() << "): ";
        for (float f : init_pos_offset) std::cout << f << " ";
        std::cout << "\n";

        std::cout << "joint_dir (" << joint_dir.size() << "): ";
        for (float f : joint_dir) std::cout << f << " ";
        std::cout << "\n";
        
        // float init_pos_offset[31] ;
        // float joint_dir[31] ;
        // assert()
        // for(int i=0;i<31;i++)
        // {
        //     init_pos_offset[i] = init_pos_offset[i];

        // }

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


 
    std::vector<float> load_joint_array(const json& joint_section) {
        std::vector<float> result;

        for (const auto& [key, arr] : joint_section.items()) {
            for (float value : arr) {
                result.push_back(value);
            }
        }

        return result;
    }

    ~CR1_PRO_WBC_HardwareInterface(){}

    virtual void Start(){
        time_stamp_ = GetTimestampMs();
        ResetPositionOffset();
    }

    virtual void Stop(){
    }
};





#endif
