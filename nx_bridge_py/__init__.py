"""
NX Bridge Python SDK for Lite3 quadruped robot remote control.

Provides a clean Python API for communicating with the RK3588 robot-side
bridge over UDP. Use this on the Jetson NX to receive sensor data and send
joint commands at ~50 Hz.

Quick start:
    from nx_bridge_py import NxBridge, NxJointCommand, NxRobotState

    def policy(state: NxRobotState) -> NxJointCommand:
        cmd = NxJointCommand()
        cmd.set_standing_pose()
        return cmd

    bridge = NxBridge(recv_port=31001, robot_ip="192.168.1.100", send_port=31002)
    bridge.start(policy, period_ms=20)
    # ... Ctrl+C to stop ...
"""

from .nx_bridge import (
    NxBridge,
    NxRobotState,
    NxJointCommand,
    ObservationCallback,
    ActionCallback,
    PolicyCallback,
    SHUTDOWN_SEQUENCE,
    DEFAULT_SENSOR_PORT,
    DEFAULT_COMMAND_PORT,
)

__all__ = [
    "NxBridge",
    "NxRobotState",
    "NxJointCommand",
    "ObservationCallback",
    "ActionCallback",
    "PolicyCallback",
    "SHUTDOWN_SEQUENCE",
    "DEFAULT_SENSOR_PORT",
    "DEFAULT_COMMAND_PORT",
]
