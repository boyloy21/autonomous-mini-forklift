#include "forklift_mission_manager/mission_state_machine.hpp"

namespace forklift_mission_manager
{

    std::string missionStateToString(MissionState s)
    {
        switch (s)
        {
            case MissionState::IDLE: return "IDLE";
            case MissionState::RECEIVED: return "RECEIVED";
            case MissionState::VALIDATING: return "VALIDATING";
            case MissionState::EXECUTING: return "EXECUTING";
            case MissionState::WAITING_TASK_RESULT: return "WAITING_TASK_RESULT";
            case MissionState::NEXT_TASK: return "NEXT_TASK";
            case MissionState::COMPLETED: return "COMPLETED";
            case MissionState::FAILED: return "FAILED";
            case MissionState::CANCELED: return "CANCELED";
            case MissionState::TIMEOUT: return "TIMEOUT";
            case MissionState::SAFETY_STOP: return "SAFETY_STOP";
            default: return "UNKNOWN";
        }
    }

    bool MissionStateMachine::isTerminal() const
    {
        return state_ == MissionState::COMPLETED || state_ == MissionState::FAILED ||
               state_ == MissionState::CANCELED || state_ == MissionState::TIMEOUT ||
               state_ == MissionState::SAFETY_STOP;
    }
}