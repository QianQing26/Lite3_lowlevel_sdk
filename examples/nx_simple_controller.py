#!/usr/bin/env python3
"""
Example Jetson NX program using the NxBridge Python SDK.

Demonstrates two usage modes:
  1. Dual-callback: separate observation and action callbacks
  2. Single-callback: one function that receives obs and returns action

Usage:
    python nx_simple_controller.py [robot_ip] [recv_port] [send_port]

Defaults: robot_ip=127.0.0.1, recv_port=31001, send_port=31002
"""

import sys
import time
import math

# Add parent dir to path so we can import nx_bridge_py directly
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from nx_bridge_py import NxBridge, NxRobotState, NxJointCommand


# ============================================================================
# Fixed standing pose for Lite3 (12 joints, 4 legs × 3 DOF)
# ============================================================================
# Joint order: FL_HipX, FL_HipY, FL_Knee, FR_HipX, FR_HipY, FR_Knee,
#               HL_HipX, HL_HipY, HL_Knee, HR_HipX, HR_HipY, HR_Knee

STAND_POSE = {
    "hip_x":  0.00,
    "hip_y": -0.65,
    "knee":   1.30,
}


# ============================================================================
# Example 1: Dual-callback API
# ============================================================================

class DualCallbackExample:
    """Separate observation and action callbacks."""

    def __init__(self):
        self.obs_count = 0
        self.cmd = NxJointCommand()
        self.cmd.set_standing_pose(**STAND_POSE)

    def on_observation(self, state: NxRobotState):
        self.obs_count += 1
        if self.obs_count % 50 == 0:
            rpy_deg = tuple(v * 180.0 / math.pi for v in state.rpy)
            print(f"[Example] seq={state.seq} state={state.current_state} "
                  f"rpy=({rpy_deg[0]:.1f}, {rpy_deg[1]:.1f}, {rpy_deg[2]:.1f}) deg "
                  f"cmd_vel=({state.cmd_vel[0]:.2f}, {state.cmd_vel[1]:.2f}, {state.cmd_vel[2]:.2f})")

    def get_action(self) -> NxJointCommand:
        return self.cmd


def run_dual_callback(robot_ip: str, recv_port: int, send_port: int):
    print("\n=== Dual-Callback Example ===")
    example = DualCallbackExample()

    bridge = NxBridge(recv_port=recv_port, robot_ip=robot_ip, send_port=send_port)
    bridge.start(example.on_observation, example.get_action, period_ms=20)

    print("Running... Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        bridge.stop()


# ============================================================================
# Example 2: Single-callback API (simpler, for stateful policies)
# ============================================================================

def run_single_callback(robot_ip: str, recv_port: int, send_port: int):
    print("\n=== Single-Callback Example ===")
    tick = [0]  # use list for mutable closure

    def policy(state: NxRobotState) -> NxJointCommand:
        tick[0] += 1
        if tick[0] % 50 == 0:
            rpy_deg = state.rpy[0] * 180.0 / math.pi
            print(f"[Policy] tick={tick[0]} rpy.x={rpy_deg:.1f} deg")

        cmd = NxJointCommand()
        cmd.set_standing_pose(**STAND_POSE)
        return cmd

    bridge = NxBridge(recv_port=recv_port, robot_ip=robot_ip, send_port=send_port)
    bridge.start(policy, period_ms=20)

    print("Running... Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        bridge.stop()


# ============================================================================
# Example 3: Sinusoidal motion (demo of dynamic commands)
# ============================================================================

def run_sine_wave_example(robot_ip: str, recv_port: int, send_port: int):
    """Demonstrate a simple sinusoidal knee motion while holding standing pose."""
    print("\n=== Sine Wave Motion Example ===")
    start_time = time.time()

    def policy(state: NxRobotState) -> NxJointCommand:
        elapsed = time.time() - start_time
        # Small sinusoidal perturbation on knee joints (±0.1 rad at 0.5 Hz)
        knee_offset = 0.1 * math.sin(2.0 * math.pi * 0.5 * elapsed)

        cmd = NxJointCommand()
        sp = STAND_POSE
        poses = [
            sp["hip_x"],  sp["hip_y"],  sp["knee"] + knee_offset,   # FL
           -sp["hip_x"],  sp["hip_y"],  sp["knee"] + knee_offset,   # FR
            sp["hip_x"],  sp["hip_y"],  sp["knee"] + knee_offset,   # HL
           -sp["hip_x"],  sp["hip_y"],  sp["knee"] + knee_offset,   # HR
        ]
        cmd.joint_pos_des = list(poses)
        cmd.set_gains(kp=40.0, kd=1.5)  # slightly stiffer for dynamic motion
        return cmd

    bridge = NxBridge(recv_port=recv_port, robot_ip=robot_ip, send_port=send_port)
    bridge.start(policy, period_ms=20)

    print("Running sine wave motion... Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        bridge.send_shutdown()
        bridge.stop()


# ============================================================================
# Main
# ============================================================================

def main():
    robot_ip  = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    recv_port = int(sys.argv[2]) if len(sys.argv) > 2 else 31001
    send_port = int(sys.argv[3]) if len(sys.argv) > 3 else 31002

    print("=== Lite3 NX Python Controller ===")
    print(f"Robot IP : {robot_ip}")
    print(f"Recv port: {recv_port} (SensorPacket from RK3588)")
    print(f"Send port: {send_port} (CommandPacket to RK3588)")
    print(f"Stand pose: {STAND_POSE}")

    # Run the dual-callback example by default.
    # Change to run_single_callback or run_sine_wave_example to test other modes.
    run_dual_callback(robot_ip, recv_port, send_port)


if __name__ == "__main__":
    main()
