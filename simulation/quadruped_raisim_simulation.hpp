#ifndef QUADRUPED_RAISIM_SIMULATION_HPP_
#define QUADRUPED_RAISIM_SIMULATION_HPP_

#include "simulation/raisim_interface.hpp"

namespace interface{
class QuadrupedRaisimSimulation : public RaisimInterface{
public:
    QuadrupedRaisimSimulation(const std::string& ak, const std::string& up, const std::string& rn="x30_sim"):RaisimInterface(rn, 12){
        this->SetActivationKey(ak);
        this->SetUrdfModelPath(up);

        VecXd init_joint_pos(dof_num_+7);
        double yaw0 = 0.0 * M_PI / 2.;
        init_joint_pos << 0, 0, 0.65, 
                    cos(yaw0 / 2), 0, 0, sin(yaw0 / 2), 
                    0, -0.5, 2,
                    0, -0.5, 2,
                    0, -0.5, 2,
                    0, -0.5, 2;
        this->SetInitJointPos(init_joint_pos);
    }
    ~QuadrupedRaisimSimulation(){

    }
};
};
#endif