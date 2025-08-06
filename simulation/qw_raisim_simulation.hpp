/**
 * @file qw_raisim_simulation.hpp
 * @brief raisim simulation for quadruped_wheel robot
 * @author mazunwang
 * @version 1.0
 * @date 2024-08-07
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef QUADRUPED_WHEEL_RAISIM_SIMULATION_HPP_
#define QUADRUPED_WHEEL_RAISIM_SIMULATION_HPP_

#include "simulation/raisim_interface.hpp"


namespace interface{
class QwRaisimSimulation : public RaisimInterface{
public:
    QwRaisimSimulation(const std::string& ak, const std::string& up, const std::string& rn="k1w_sim"):RaisimInterface(rn, 16){
        this->SetActivationKey(ak);
        this->SetUrdfModelPath(up);

        VecXd init_joint_pos(dof_num_+7);
        double yaw0 = 0.0 * M_PI / 2.;
        init_joint_pos << 0, 0, 0.65, 
                    cos(yaw0 / 2), 0, 0, sin(yaw0 / 2), 
                    0, -0.5, 2, 0,
                    0, -0.5, 2, 0,
                    0, -0.5, 2, 0,
                    0, -0.5, 2, 0;
        this->SetInitJointPos(init_joint_pos);
    }
    ~QwRaisimSimulation(){

    }


    
};
};


#endif