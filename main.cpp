#include "humanoid/humanoid_state_machine.hpp"

#include "dr_logger.hpp"
#include "drdds/core/drdds_core.h"

using namespace types;
MotionStateFeedback StateBase::msfb_ = MotionStateFeedback();
// #define BACKWARD_HAS_DW 1
// #include "backward.hpp"
// namespace backward{
//     backward::SignalHandling sh;
// }

int main(){
    std::cout << "State Machine Start Running" << std::endl;

    DrDDSManager::Init({0}, "50/1", "ECAN");
    // std::shared_ptr<StateMachineBase> fsm = std::make_shared<humanoid::HumanoidStateMachine>(RobotName::CR1LEG, RemoteCommandType::kKeyBoard);
    std::shared_ptr<StateMachineBase> fsm = std::make_shared<humanoid::HumanoidStateMachine>(RobotName::CR1Pro, RemoteCommandType::kKeyBoard);

    fsm->Start();
    fsm->Run();
    fsm->Stop(); 
    return 0;
}