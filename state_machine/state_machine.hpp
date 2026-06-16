/**
 * @file state_machine.hpp
 * @brief Lite3 robot control state machine — deployment version (remote NX bridge)
 * @author mazunwang (original), adapted for remote NX deployment
 * @version 2.0
 * @date 2024-05-29
 *
 * @copyright Copyright (c) 2024  DeepRobotics
 *
 * State transitions: Idle → StandUp → RLControl(Remote) → JointDamping → Idle
 *
 * RL control commands are received from a remote Jetson NX via CommBridge.
 * PD tracking is performed by the robot's motor controller at 2 kHz.
 */

#pragma once

#include "state_base.h"
#include "idle_state.hpp"
#include "standup_state.hpp"
#include "joint_damping_state.hpp"
#include "rl_handshake_state.hpp"
#include "rl_control_state_remote.hpp"
#include "retroid_gamepad_interface.hpp"
#include "hardware/hardware_interface.hpp"
#include "data_streaming.hpp"

class StateMachine{
private:
    std::shared_ptr<StateBase> current_controller_;
    std::shared_ptr<StateBase> idle_controller_;
    std::shared_ptr<StateBase> standup_controller_;
    std::shared_ptr<StateBase> handshake_controller_;
    std::shared_ptr<StateBase> rl_controller_;
    std::shared_ptr<StateBase> joint_damping_controller_;

    StateName current_state_name_, next_state_name_;

    std::shared_ptr<UserCommandInterface> uc_ptr_;
    std::shared_ptr<RobotInterface> ri_ptr_;
    std::shared_ptr<ControlParameters> cp_ptr_;

    std::shared_ptr<DataStreaming> ds_ptr_;

    void GetDataStreaming(){
        if(!ri_ptr_) return;
        VecXf pos = ri_ptr_->GetJointPosition();
        VecXf vel = ri_ptr_->GetJointVelocity();
        VecXf tau = ri_ptr_->GetJointTorque();
        Vec3f rpy = ri_ptr_->GetImuRpy();
        Vec3f acc = ri_ptr_->GetImuAcc();
        Vec3f omg = ri_ptr_->GetImuOmega();
        MatXf jc = ri_ptr_->GetJointCommand();

        ds_ptr_->InsertInterfaceTime(ri_ptr_->GetInterfaceTimeStamp());
        ds_ptr_->InsertJointData("q", pos);
        ds_ptr_->InsertJointData("dq", vel);
        ds_ptr_->InsertJointData("tau", tau);
        ds_ptr_->InsertJointData("q_cmd", jc.col(1));
        ds_ptr_->InsertJointData("tau_ff", jc.col(4));

        ds_ptr_->InsertImuData("rpy", rpy);
        ds_ptr_->InsertImuData("acc", acc);
        ds_ptr_->InsertImuData("omg", omg);

        if(!uc_ptr_) return;
        auto cmd = uc_ptr_->GetUserCommand();
        ds_ptr_->InsertCommandData("target_mode", float(cmd.target_mode));

        ds_ptr_->InsertStateData("current_state", StateBase::msfb_.current_state);

        ds_ptr_->SendData();
    }

    std::shared_ptr<StateBase> GetNextStatePtr(StateName state_name){
        switch(state_name){
            case StateName::kInvalid:{
                return nullptr;
            }
            case StateName::kIdle:{
                return idle_controller_;
            }
            case StateName::kStandUp:{
                return standup_controller_;
            }
            case StateName::kRLHandshake:{
                return handshake_controller_;
            }
            case StateName::kRLControl:{
                return rl_controller_;
            }
            case StateName::kJointDamping:{
                return joint_damping_controller_;
            }
            default:{
                std::cerr << "error state name" << std::endl;
            }
        }
        return nullptr;
    }
public:
    StateMachine(RobotType robot_type){
        // User input: Retroid gamepad (default for real hardware).
        // To use keyboard instead, replace with:
        //   uc_ptr_ = std::make_shared<KeyboardInterface>();
        uc_ptr_ = std::make_shared<RetroidGamepadInterface>(12121);

        // Robot interface: always real hardware via Lite3 MotionSDK
        ri_ptr_ = std::make_shared<HardwareInterface>("Lite3");
        cp_ptr_ = std::make_shared<ControlParameters>(robot_type);

        std::shared_ptr<ControllerData> data_ptr = std::make_shared<ControllerData>();
        data_ptr->ri_ptr = ri_ptr_;
        data_ptr->uc_ptr = uc_ptr_;
        data_ptr->cp_ptr = cp_ptr_;
        ds_ptr_ = std::make_shared<DataStreaming>(false, false);
        data_ptr->ds_ptr = ds_ptr_;

        idle_controller_ = std::make_shared<IdleState>(robot_type, "idle_state", data_ptr);
        standup_controller_ = std::make_shared<StandUpState>(robot_type, "standup_state", data_ptr);
        handshake_controller_ = std::make_shared<RLHandshakeState>(robot_type, "rl_handshake", data_ptr);
        rl_controller_ = std::make_shared<RLControlStateRemote>(robot_type, "rl_control", data_ptr);
        joint_damping_controller_ = std::make_shared<JointDampingState>(robot_type, "joint_damping", data_ptr);

        current_controller_ = idle_controller_;
        current_state_name_ = kIdle;
        next_state_name_ = kIdle;

        ri_ptr_->Start();
        std::cout << "Robot interface started" << std::endl;
        uc_ptr_->Start();

        current_controller_->OnEnter();
    }
    ~StateMachine(){}

    void Run(){
        int cnt = 0;
        static double time_record = 0;
        while(true){
            if(ri_ptr_->GetInterfaceTimeStamp()!= time_record){
                time_record = ri_ptr_->GetInterfaceTimeStamp();
                current_controller_ -> Run();

                if(current_controller_->LoseControlJudge()) next_state_name_ = StateName::kJointDamping;
                else next_state_name_ = current_controller_ -> GetNextStateName();

                if(next_state_name_ != current_state_name_){
                    current_controller_ -> OnExit();
                    std::cout << current_controller_ -> state_name_ << " ------------> ";
                    current_controller_ = GetNextStatePtr(next_state_name_);
                    std::cout << current_controller_ -> state_name_ << std::endl;
                    current_controller_ ->OnEnter();
                    current_state_name_ = next_state_name_;
                }
                ++cnt;
                this->GetDataStreaming();
            }
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }

        uc_ptr_->Stop();
        ri_ptr_->Stop();
    }

};
