/**
 * @file rl_handshake_state.hpp
 * @brief Handshake state — establishes connection with NX before entering RL control
 *
 * On entry (after StandUp completes):
 *   1. Creates CommBridge, starts UDP send/recv threads
 *   2. Sends SensorPacket(state=kRLHandshake) so NX knows to prepare
 *   3. Waits for CommandPacket with heartbeat==HEARTBEAT_READY from NX
 *
 * On READY received:
 *   → transitions to RLControl (CommBridge stays running, shared via ControllerData)
 *
 * On timeout (3 s):
 *   → transitions to JointDamping
 */

#pragma once

#include "state_base.h"
#include "comm_bridge.hpp"

#include <cmath>

class RLHandshakeState : public StateBase
{
private:
    double enter_time_ = 0.0;
    static constexpr double TIMEOUT_S = 3.0;

public:
    RLHandshakeState(const RobotType &robot_type,
                     const std::string &state_name,
                     std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_type, state_name, data_ptr) {}

    virtual void OnEnter() override
    {
        // Always create a fresh CommBridge and start it.
        // Stop/destroy the old one first to ensure clean state
        // (avoids stale has_ready flag, socket reuse issues).
        if (data_ptr_->comm_ptr)
        {
            data_ptr_->comm_ptr->Stop();
        }
        data_ptr_->comm_ptr = std::make_shared<CommBridge>(NX_IP_ADDRESS);
        data_ptr_->comm_ptr->Start();

        enter_time_ = ri_ptr_->GetInterfaceTimeStamp();

        StateBase::msfb_.UpdateCurrentState(RobotMotionState::RLHandshakeMode);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);

        std::cout << "[RLHandshake] Waiting for NX READY at "
                  << NX_IP_ADDRESS << " ("
                  << TIMEOUT_S << " s timeout)..." << std::endl;
    }

    virtual void OnExit() override
    {
        // CommBridge stays running — RLControlRemote will take over
        std::cout << "[RLHandshake] Exited" << std::endl;
    }

    virtual void Run() override
    {
        // Send sensor data so NX knows we are trying to handshake.
        // Use the current motion state (= RLHandshakeMode) in the packet,
        // so the NX can identify this as a handshake request.
        RobotBasicState rbs;
        rbs.base_rpy   = ri_ptr_->GetImuRpy();
        rbs.base_omega = ri_ptr_->GetImuOmega();
        rbs.base_acc   = ri_ptr_->GetImuAcc();
        rbs.joint_pos  = ri_ptr_->GetJointPosition();
        rbs.joint_vel  = ri_ptr_->GetJointVelocity();
        rbs.joint_tau  = ri_ptr_->GetJointTorque();

        UserCommand uc = uc_ptr_->GetUserCommand();
        rbs.cmd_vel_normlized = Vec3f(uc.forward_vel_scale,
                                      uc.side_vel_scale,
                                      uc.turnning_vel_scale);

        uint32_t ts_ms = static_cast<uint32_t>(ri_ptr_->GetInterfaceTimeStamp() * 1000.0);
        data_ptr_->comm_ptr->UpdateSensorData(rbs, uc,
                                               RobotMotionState::RLHandshakeMode,
                                               ts_ms);
    }

    virtual bool LoseControlJudge() override
    {
        // User aborted
        if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping))
        {
            std::cout << "[RLHandshake] User requested JointDamping" << std::endl;
            return true;
        }

        // Posture unsafe
        Vec3f rpy = ri_ptr_->GetImuRpy();
        if (std::fabs(rpy(0)) > 30.0 / 180.0 * M_PI ||
            std::fabs(rpy(1)) > 45.0 / 180.0 * M_PI)
        {
            std::cout << "[RLHandshake] Posture unsafe" << std::endl;
            return true;
        }

        // Timeout
        double elapsed = ri_ptr_->GetInterfaceTimeStamp() - enter_time_;
        if (elapsed > TIMEOUT_S)
        {
            std::cout << "[RLHandshake] Timeout (" << elapsed
                      << " s) — no READY from NX" << std::endl;
            return true;
        }

        return false;
    }

    virtual StateName GetNextStateName() override
    {
        // Check if NX has sent READY
        if (data_ptr_->comm_ptr && data_ptr_->comm_ptr->HasReceivedReady())
        {
            std::cout << "[RLHandshake] READY received from NX → RLControl" << std::endl;
            return StateName::kRLControl;
        }
        return StateName::kRLHandshake;
    }
};
