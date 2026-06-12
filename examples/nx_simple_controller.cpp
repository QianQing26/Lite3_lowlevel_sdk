/**
 * @file nx_simple_controller.cpp
 * @brief Example Jetson NX program using the NxBridge API
 *
 * This demonstrates the simplest possible NX-side controller:
 *   1. Receives sensor data from RK3588
 *   2. Prints a summary once per second
 *   3. Sends a fixed standing-pose command (position hold)
 *
 * Build (on Jetson NX or x86 for testing):
 *   g++ -std=c++17 -I.. -o nx_simple_controller nx_simple_controller.cpp \
 *       ../nx_bridge/nx_bridge.cpp -lpthread
 *
 * Or via CMake: -DBUILD_NX_SDK=ON
 *
 * Usage:
 *   ./nx_simple_controller [robot_ip] [recv_port] [send_port]
 *
 * Defaults: robot_ip=127.0.0.1, recv_port=31001, send_port=31002
 */

#include "nx_bridge_api.hpp"

#include <iostream>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <chrono>
#include <thread>

// ============================================================================
// Fixed standing pose for Lite3 (12 joints, 4 legs × 3 DOF each)
// ============================================================================
// Joint order: FL_HipX, FL_HipY, FL_Knee, FR_HipX, FR_HipY, FR_Knee,
//               HL_HipX, HL_HipY, HL_Knee, HR_HipX, HR_HipY, HR_Knee
//
// These are approximate standing-pose joint angles (radians).
// Adjusted for your specific robot if needed.

struct StandPose {
    float hip_x  =  0.0f;     // abduction — centered
    float hip_y  = -0.65f;    // hip pitch
    float knee   =  1.30f;    // knee
};

static const StandPose kStand = {0.0f, -0.65f, 1.30f};

// ============================================================================
// Example 1: Two-callback API (separate observation and action callbacks)
// ============================================================================

static NxJointCommand s_fixed_pose;   // pre-computed command to send every cycle
static int s_obs_count = 0;

void OnObservation(const NxRobotState& state) {
    s_obs_count++;

    // Print summary once per second (at ~50 Hz we get ~50 obs/s)
    if (s_obs_count % 50 == 0) {
        std::cout << "[Example] seq=" << state.seq
                  << " state=" << state.current_state
                  << " rpy=("
                  << std::fixed << std::setprecision(2)
                  << state.rpy[0] * 180.0 / M_PI << ", "
                  << state.rpy[1] * 180.0 / M_PI << ", "
                  << state.rpy[2] * 180.0 / M_PI << ") deg"
                  << " cmd_vel=(" << state.cmd_vel[0] << ", "
                  << state.cmd_vel[1] << ", " << state.cmd_vel[2] << ")"
                  << std::endl;
    }
}

NxJointCommand GetAction() {
    // Always return the same fixed standing pose.
    // RK3588 applies zero-order hold between NX updates.
    return s_fixed_pose;
}

void RunTwoCallbackExample(const std::string& robot_ip, int recv_port, int send_port) {
    std::cout << "\n=== Two-Callback Example ===" << std::endl;

    // Configure default PD gains (applied if the returned command has zeros)
    NxBridge::SetDefaultGains(30.0f, 1.0f);

    // Build the fixed standing-pose command
    std::memset(&s_fixed_pose, 0, sizeof(s_fixed_pose));
    const StandPose& sp = kStand;
    // FL: {hip_x, hip_y, knee}
    s_fixed_pose.joint_pos_des[0] = sp.hip_x;   s_fixed_pose.joint_pos_des[1] = sp.hip_y;   s_fixed_pose.joint_pos_des[2] = sp.knee;
    // FR: {-hip_x, hip_y, knee}  (mirror image)
    s_fixed_pose.joint_pos_des[3] = -sp.hip_x;  s_fixed_pose.joint_pos_des[4] = sp.hip_y;   s_fixed_pose.joint_pos_des[5] = sp.knee;
    // HL: {hip_x, hip_y, knee}
    s_fixed_pose.joint_pos_des[6] = sp.hip_x;   s_fixed_pose.joint_pos_des[7] = sp.hip_y;   s_fixed_pose.joint_pos_des[8] = sp.knee;
    // HR: {-hip_x, hip_y, knee}
    s_fixed_pose.joint_pos_des[9] = -sp.hip_x;  s_fixed_pose.joint_pos_des[10] = sp.hip_y;  s_fixed_pose.joint_pos_des[11] = sp.knee;

    // Set PD gains — these are the same for all joints
    for (int i = 0; i < 12; ++i) {
        s_fixed_pose.kp[i]     = 30.0f;
        s_fixed_pose.kd[i]     = 1.0f;
        s_fixed_pose.tau_ff[i] = 0.0f;
    }

    NxBridge bridge(recv_port, robot_ip, send_port);
    bridge.Start(OnObservation, GetAction, 20.0f);  // 50 Hz

    std::cout << "Running... Press Ctrl+C to stop." << std::endl;

    // Keep the main thread alive while the bridge runs in background threads
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    bridge.Stop();
}

// ============================================================================
// Example 2: Single-callback API (simpler, for stateful policies)
// ============================================================================

void RunSingleCallbackExample(const std::string& robot_ip, int recv_port, int send_port) {
    std::cout << "\n=== Single-Callback Example ===" << std::endl;

    int tick = 0;

    // The single-callback version: function receives latest obs, returns action.
    auto policy = [&tick](const NxRobotState& state) -> NxJointCommand {
        tick++;
        NxJointCommand cmd;
        std::memset(&cmd, 0, sizeof(cmd));

        // Print every 50th tick
        if (tick % 50 == 0) {
            std::cout << "[Policy] tick=" << tick
                      << " rpy.x=" << state.rpy[0] * 180.0 / M_PI << " deg"
                      << std::endl;
        }

        // Fill with standing pose (same as two-callback example)
        const StandPose& sp = kStand;
        float poses[12] = {
            sp.hip_x, sp.hip_y, sp.knee,
            -sp.hip_x, sp.hip_y, sp.knee,
            sp.hip_x, sp.hip_y, sp.knee,
            -sp.hip_x, sp.hip_y, sp.knee
        };
        std::memcpy(cmd.joint_pos_des, poses, sizeof(poses));
        for (int i = 0; i < 12; ++i) {
            cmd.kp[i] = 30.0f;
            cmd.kd[i] = 1.0f;
        }
        return cmd;
    };

    NxBridge bridge(recv_port, robot_ip, send_port);
    bridge.Start(policy, 20.0f);  // 50 Hz

    std::cout << "Running... Press Ctrl+C to stop." << std::endl;

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    bridge.Stop();
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    std::string robot_ip = (argc > 1) ? argv[1] : "127.0.0.1";
    int recv_port = (argc > 2) ? std::atoi(argv[2]) : 31001;
    int send_port = (argc > 3) ? std::atoi(argv[3]) : 31002;

    std::cout << "=== Lite3 NX Simple Controller ===" << std::endl;
    std::cout << "Robot IP : " << robot_ip << std::endl;
    std::cout << "Recv port: " << recv_port << " (SensorPacket from RK3588)" << std::endl;
    std::cout << "Send port: " << send_port << " (CommandPacket to RK3588)" << std::endl;
    std::cout << "Stand pose: hip_x=" << kStand.hip_x
              << " hip_y=" << kStand.hip_y
              << " knee=" << kStand.knee << std::endl;

    // Run the two-callback example by default.
    // Change to RunSingleCallbackExample to test the simpler API.
    RunTwoCallbackExample(robot_ip, recv_port, send_port);

    return 0;
}
