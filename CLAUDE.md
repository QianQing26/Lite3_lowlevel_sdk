# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Lite3 Lowlevel SDK is the low-level control runtime for the Lite3 quadruped robot, running on the RK3588 motion computer. It manages the state machine, hardware communication, 2 kHz PD control, and safety protection, while forwarding sensor data to a high-level controller (e.g. Jetson NX) and executing joint position commands received over UDP.

The high-level control API is provided by the separate [Lite3 Highlevel SDK](https://github.com/DeepRoboticsLab/Lite3_highlevel_sdk) repository.

## Build Commands

### Real Hardware (ARM — RK3588)
```bash
mkdir build && cd build
cmake .. -DBUILD_PLATFORM=arm
make -j
```

### Development (x86 — compile check only)
```bash
mkdir build && cd build
cmake .. -DBUILD_PLATFORM=x86
make -j
```

### CMake Options
- `-DBUILD_PLATFORM`: `arm` (robot hardware, default) or `x86` (dev compile check)
- `-DSEND_REMOTE=ON`: auto-SCP binary to robot after build
- `-DUSE_MOLD_LINKER=ON`: use mold linker for faster linking

## Running

**Real hardware:**
Connect to robot WiFi (`Lite*******`, password `12345678`), deploy and run:
```bash
scp -r ~/Lite3_lowlevel_sdk ysc@192.168.2.1:~/
ssh ysc@192.168.2.1
cd Lite3_lowlevel_sdk/build
./rl_deploy
```

**NX side:**
On the Jetson NX, run your policy with the NxBridge SDK (see `nx_bridge/nx_bridge_api.hpp`).
Communication ports: RK3588→NX on 31001, NX→RK3588 on 31002.

## Architecture

### State Machine (`state_machine/`)
The central control loop at 2 kHz. States transition linearly: **Idle → StandUp → RLControl(Remote) → JointDamping → Idle**

- `state_machine.hpp`: Orchestrates transitions, user input, data streaming. Always uses HardwareInterface + RetroidGamepadInterface.
- `state_base.h`: Abstract base with `Run()`, `OnEnter()`, `OnExit()`, `GetNextStateName()`, `LoseControlJudge()`
- `rl_control_state_remote.hpp`: Sends sensor data to NX via CommBridge, receives joint commands, applies 2 kHz zero-order hold. Safety: posture check (roll ±30°/pitch ±45°) + communication timeout (40 ms) → JointDamping
- `standup_state.hpp`: Two-phase cubic-spline stand-up sequence (~3 s)
- `idle_state.hpp`: Validates sensor data before allowing stand-up
- `joint_damping_state.hpp`: Passive damping fallback (3 s → Idle)

### Communication Layer (`communication/`)
- `nx_network_codes.hpp`: Binary packet structs — `SensorPacket` (204 B, RK3588→NX) and `CommandPacket` (248 B, NX→RK3588). Integer fields use network byte order.
- `comm_bridge.hpp`: RK3588-side UDP bridge. Send thread (~50 Hz, timerfd+epoll), recv thread (blocking). Thread-safe command access via mutex. Timeout detection at 40 ms.

### Interface Layer (`interface/`)
- `robot_interface.h`: Abstract base — `GetJointPosition/Velocity/Torque()`, `GetImuRpy/Acc/Omega()`, `SetJointCommand(12×5 matrix)`
- `hardware/hardware_interface.hpp`: Real robot via Lite3 MotionSDK (UDP to motor controller)
- `user_command_interface.h`: Abstract base for gamepad/keyboard
- `retroid_gamepad_interface.hpp`: Retroid gamepad (default for hardware)

### Types (`types/`)
- `common_types.h`: Eigen aliases (`VecXf`, `MatXf`), `RobotBasicState`, `RobotAction`, `UserCommand`, `MotionStateFeedback`
- `custom_types.h`: Enums (`RobotType`, `RobotMotionState`, `StateName`)

### Third-Party (`third_party/`)
- `eigen/`: Header-only linear algebra
- `Lite3_MotionSDK/`: Robot hardware SDK (`.so` for x86 and aarch64)
- `gamepad/`: Gamepad SDK (Retroid + Skydroid support)

## Key Timing
- Main control loop: 2 kHz (500 μs sleep)
- NX communication: ~50 Hz send/recv
- PD control: by robot motor controller via MotionSDK
- Communication timeout: 40 ms (2 missed packets at 50 Hz)

## Key Files for Common Tasks

| Task | File |
|------|------|
| Change NX IP address | `state_machine/rl_control_state_remote.hpp` (`NX_IP_ADDRESS` macro) |
| Tune safety limits | `rl_control_state_remote.hpp` `PostureUnsafeCheck()` |
| Adjust timeout | `communication/comm_bridge.hpp` `timeout_ms_` |
| Change control gains | `state_machine/parameters/` |
| Switch to keyboard control | `state_machine/state_machine.hpp` (comment in constructor) |
| Write NX-side policy | Use [Lite3 Highlevel SDK](https://github.com/DeepRoboticsLab/Lite3_highlevel_sdk) |
