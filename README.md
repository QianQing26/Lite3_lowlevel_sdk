# Lite3 Lowlevel SDK

[![Discord](https://img.shields.io/badge/-Discord-5865F2?style=flat&logo=Discord&logoColor=white)](https://discord.gg/gdM9mQutC8)

Lite3 四足机器人的**底层控制 SDK**，运行在 RK3588 运动主机上。

负责所有底层工作——状态机管理、硬件通信、PD 闭环、安全保护——并通过 UDP 向上层控制器（如 Jetson NX）转发传感器数据和手柄指令，接收并执行关节位置指令。上层只需关注策略与规划。

---

## 架构

```
┌──────────────────────────────────────────────────┐
│              上层控制器 (NX / PC)                  │
│          RL 策略 · 轨迹规划 · 感知                  │
│     通过 UDP 收发 SensorPacket / CommandPacket     │
└──────────────────┬───────────────────────────────┘
                   │  UDP :31001 ← → :31002
┌──────────────────┴───────────────────────────────┐
│           Lite3 Lowlevel SDK (RK3588)             │
│                                                   │
│  State Machine (2 kHz)                             │
│  ├ Idle         — 传感器校验，等待站立指令           │
│  ├ StandUp      — 三次样条站立序列 (~3 s)            │
│  ├ RLControl    — 接收上层指令，PD 闭环，数据转发     │
│  └ JointDamping — 安全回退，关节阻尼                 │
│                                                   │
│  CommBridge     — UDP 收发 (send ~50 Hz, recv 阻塞) │
│  HardwareInterface — Lite3 MotionSDK              │
│  GamepadInterface  — Retroid 手柄                 │
│  Safety — 姿态保护 + 通信超时 + 传感器异常检测        │
└──────────────────┬───────────────────────────────┘
                   │  MotionSDK (UDP)
┌──────────────────┴───────────────────────────────┐
│              Lite3 机器人硬件                       │
│         关节电机 · IMU · 足端传感器                  │
└──────────────────────────────────────────────────┘
```

### 状态转移

```
Idle ──→ StandUp ──→ RLControl ──→ JointDamping
          ↑ 超时 / 姿态异常 / 用户指令  │
          └──────────────────────────┘
                       │
                       ↓ (3 s 后)
                     Idle
```

### 数据流

| 方向 | 端口 | 内容 | 频率 |
|------|------|------|------|
| SDK → 上层 | 31001 | 关节位置/速度/力矩、IMU (rpy/acc/omega)、手柄指令、状态 | ~50 Hz |
| 上层 → SDK | 31002 | 目标关节位置/速度、PD 增益 (kp/kd)、前馈力矩 (tau_ff) | ~50 Hz |

SDK 在两次上层指令之间做**零阶保持**（ZOH），以 2 kHz 持续向电机下发 PD 指令。

---

## 快速开始

全程在 Ubuntu 系统上进行。

### 1. 编译（RK3588 / ARM aarch64）

```bash
git clone --recurse-submodule https://github.com/DeepRoboticsLab/Lite3_lowlevel_sdk.git
cd Lite3_lowlevel_sdk
mkdir build && cd build
cmake .. -DBUILD_PLATFORM=arm
make -j
```

### 2. 编译（x86，仅编译检查）

```bash
cmake .. -DBUILD_PLATFORM=x86
make -j
```

### 3. CMake 选项

| 选项 | 默认值 | 说明 |
|------|--------|------|
| `BUILD_PLATFORM` | `arm` | `arm` = RK3588 真机，`x86` = 仅编译检查 |
| `SEND_REMOTE` | `OFF` | 编译后自动 scp 到机器人 |
| `BUILD_NX_SDK` | `OFF` | 同时编译上层通信示例（C++/Python 参考实现） |

### 4. 部署与运行

```bash
# 电脑和手柄均连接机器狗 WiFi
# SSID: Lite*******    密码: 12345678

# 传输文件到运动主机
scp -r ~/Lite3_lowlevel_sdk ysc@192.168.2.1:~/

# SSH 到运动主机
ssh ysc@192.168.2.1        # 用户名: ysc  密码: ' (单引号)

# 编译并运行
cd Lite3_lowlevel_sdk
mkdir build && cd build
cmake .. -DBUILD_PLATFORM=arm
make -j
./rl_deploy
```

### 5. 操控

默认使用 **Retroid 手柄**。如需键盘控制，修改 `state_machine/state_machine.hpp` 构造函数中的手柄初始化行。

---

## 通信协议

SDK 与上层控制器通过 **UDP 二进制协议** 通信。协议定义见 `communication/nx_network_codes.hpp`。

上层只需：
1. 监听 UDP 端口 31001，接收 `SensorPacket`（204 字节）
2. 向 UDP 端口 31002 发送 `CommandPacket`（248 字节）
3. 以 ~50 Hz 频率运行即可；SDK 负责 2 kHz PD 闭环和安全保护

### SensorPacket（SDK → 上层，204 B）

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | seq | uint32 | 序列号 |
| 4 | timestamp_ms | uint32 | 时间戳 (ms) |
| 8 | current_state | int32 | 当前状态 |
| 12 | rpy[3] | float×3 | roll, pitch, yaw (rad) |
| 24 | acc[3] | float×3 | 加速度 (m/s²) |
| 36 | omega[3] | float×3 | 角速度 (rad/s) |
| 48 | joint_pos[12] | float×12 | 关节位置 (rad) |
| 96 | joint_vel[12] | float×12 | 关节速度 (rad/s) |
| 144 | joint_tau[12] | float×12 | 关节力矩 (Nm) |
| 192 | cmd_vel[3] | float×3 | 手柄速度指令（归一化） |

### CommandPacket（上层 → SDK，248 B）

| 偏移 | 字段 | 类型 | 说明 |
|------|------|------|------|
| 0 | seq | uint32 | 序列号 |
| 4 | heartbeat | uint32 | 心跳 |
| 8 | joint_pos_des[12] | float×12 | 目标关节位置 (rad) |
| 56 | joint_vel_des[12] | float×12 | 目标关节速度 (rad/s) |
| 104 | kp[12] | float×12 | PD 比例增益 |
| 152 | kd[12] | float×12 | PD 微分增益 |
| 200 | tau_ff[12] | float×12 | 前馈力矩 (Nm) |

---

## 目录结构

```
Lite3_lowlevel_sdk/
  communication/          # 通信协议定义 + UDP 桥接
  state_machine/          # 状态机 (Idle / StandUp / RLControl / JointDamping)
  interface/              # 硬件接口 + 手柄接口
  types/                  # 数据类型 (Eigen 别名, RobotBasicState, UserCommand, …)
  utils/                  # 数学工具 + 数据记录
  nx_bridge/              # 上层 C++ 通信示例 (仅参考)
  nx_bridge_py/           # 上层 Python 通信示例 (仅参考)
  examples/               # 上层示例程序 (仅参考)
  third_party/            # 第三方依赖 (Eigen, Lite3_MotionSDK, Gamepad)
```

> `nx_bridge/`、`nx_bridge_py/`、`examples/` 仅作为通信协议的参考实现。正式的上层控制 API 将在独立仓库中提供。

---

## 安全机制

| 层级 | 触发条件 | 响应 |
|------|---------|------|
| 姿态保护 | roll > 30° 或 pitch > 45° | → JointDamping |
| 通信超时 | 40 ms 无上层指令 | → JointDamping |
| 用户中止 | 手柄阻尼键 | → JointDamping |
| 上层关闭 | seq = 0xFFFFFFFF | → JointDamping |
| 传感器异常 | 关节/IMU 超限或 NaN | Idle 中阻止站立 |

JointDamping 状态保持 3 秒后自动回到 Idle。

---

## 致谢

本项目基于 [DeepRoboticsLab/Lite3_rl_deploy](https://github.com/DeepRoboticsLab/Lite3_rl_deploy) 修改而来。

相对于原项目的主要变更：
- 移除仿真支持（PyBullet / MuJoCo / RaiSim），面向真机部署
- 移除本地 ONNX Runtime 推理，改为通过 UDP 向上层控制器转发数据并接收指令
- 新增 `communication/` 通信桥接层，定义二进制通信协议
- 精简用户输入（仅保留 Retroid 手柄），精简编译选项
- 新增上层通信协议的 C++ 与 Python 参考实现（`nx_bridge/`、`nx_bridge_py/`）

感谢原项目作者 [mazunwang](https://github.com/mazunwang) 和 [Bo (Percy) Peng](https://github.com/PercyPeng) 以及 DeepRobotics 团队提供的优秀基础框架。

---

## 许可

Copyright (c) 2024 DeepRobotics. 详见 [LICENSE](./LICENSE)。
