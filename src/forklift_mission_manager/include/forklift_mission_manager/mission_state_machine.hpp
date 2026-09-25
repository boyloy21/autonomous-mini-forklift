#ifndef FORKLIFT_MISSION_MANAGER__MISSION_STATE_MACHINE_HPP_
#define FORKLIFT_MISSION_MANAGER__MISSION_STATE_MACHINE_HPP

#include <string>

namespace forklift_mission_manager
{
    enum class MissionState
    {
        IDLE, RECEIVED, VALIDATING, EXECUTING, WAITING_TASK_RESULT, NEXT_TASK, COMPLETED,
        FAILED, CANCELED, TIMEOUT, SAFETY_STOP
    };

    std::string missionStateToString(MissionState s);

    class MissionStateMachine
    {
        public:
            MissionStateMachine() = default;
            void setState(MissionState state) {state_ = state;}
            MissionState getState() const {return state_;}
            std::string getStateString() const {return missionStateToString(state_);}
            bool isTerminal() const;
        private:
            MissionState state_{MissionState::IDLE};
    };
} // namespace forklift_mission_manager

#endif  // FORKLIFT_MISSION_MANAGER__MISSION_STATE_MACHINE_HPP_

