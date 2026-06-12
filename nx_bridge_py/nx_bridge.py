"""
NX-side Python SDK for communicating with the RK3588 Lite3 robot bridge.

Wire format (UDP, binary struct):
  - SensorPacket  (RK3588 → NX, port 31001): 204 bytes,  !IIi48f
  - CommandPacket (NX → RK3588, port 31002): 248 bytes,  !II60f

Usage (dual-callback mode):
    from nx_bridge_py import NxBridge, NxJointCommand

    def on_obs(state):
        print(f"rpy={state.rpy}")

    def get_act():
        cmd = NxJointCommand()
        cmd.set_standing_pose()
        return cmd

    bridge = NxBridge(recv_port=31001, robot_ip="192.168.1.100", send_port=31002)
    bridge.start(on_obs, get_act, period_ms=20)
    # ... Ctrl+C to stop ...

Usage (single-callback mode):
    def policy(state):
        cmd = NxJointCommand()
        cmd.set_standing_pose()
        return cmd

    bridge.start(policy, period_ms=20)
"""

import socket
import struct
import threading
import time
import dataclasses
from typing import Optional, Callable, Union

# ============================================================================
# Packet format strings (network byte order '!' = big-endian)
# ============================================================================

# SensorPacket: 2×uint32 + 1×int32 + 48×float32 = 204 bytes
_SENSOR_FMT = "!IIi48f"
_SENSOR_SIZE = struct.calcsize(_SENSOR_FMT)  # 204

# CommandPacket: 2×uint32 + 60×float32 = 248 bytes
_COMMAND_FMT = "!II60f"
_COMMAND_SIZE = struct.calcsize(_COMMAND_FMT)  # 248

# Sentinel values (mirrors nx_network_codes.hpp)
SHUTDOWN_SEQUENCE = 0xFFFFFFFF
HEARTBEAT_SHUTDOWN = 0

# Default ports
DEFAULT_SENSOR_PORT = 31001   # RK3588 → NX
DEFAULT_COMMAND_PORT = 31002  # NX → RK3588


# ============================================================================
# Data classes
# ============================================================================

@dataclasses.dataclass
class NxRobotState:
    """Sensor + gamepad data received from the robot (via RK3588)."""
    seq: int = 0
    timestamp_ms: int = 0
    current_state: int = 0
    rpy: tuple = (0.0, 0.0, 0.0)           # roll, pitch, yaw [rad]
    acc: tuple = (0.0, 0.0, 0.0)           # base acceleration [m/s²]
    omega: tuple = (0.0, 0.0, 0.0)         # angular velocity [rad/s]
    joint_pos: tuple = (0.0,) * 12         # joint positions [rad]
    joint_vel: tuple = (0.0,) * 12         # joint velocities [rad/s]
    joint_tau: tuple = (0.0,) * 12         # joint torques [Nm]
    cmd_vel: tuple = (0.0, 0.0, 0.0)       # user command velocity (normalised)

    @property
    def projected_gravity(self) -> tuple:
        """Gravity vector projected into the body frame (ZYX Euler convention).

        Uses the same computation as the RK3588-side RpyToRm + RmToProjectedGravity:
          projected_gravity = R_body_to_world^T * (0, 0, -1)

        Result:
          x =  sin(pitch)
          y = -cos(pitch) * sin(roll)
          z = -cos(pitch) * cos(roll)

        This is a standard feature in legged-robot RL policies.
        """
        import math
        cp = math.cos(self.rpy[1])   # cos(pitch)
        return (
            math.sin(self.rpy[1]),          #  sin(pitch)
            -cp * math.sin(self.rpy[0]),   # -cos(pitch) * sin(roll)
            -cp * math.cos(self.rpy[0]),   # -cos(pitch) * cos(roll)
        )

    def __repr__(self):
        return (f"NxRobotState(seq={self.seq}, state={self.current_state}, "
                f"rpy=({self.rpy[0]:.2f},{self.rpy[1]:.2f},{self.rpy[2]:.2f}))")


@dataclasses.dataclass
class NxJointCommand:
    """Joint command to send back to the robot (via RK3588)."""
    joint_pos_des: list = dataclasses.field(default_factory=lambda: [0.0] * 12)
    joint_vel_des: list = dataclasses.field(default_factory=lambda: [0.0] * 12)
    kp: list = dataclasses.field(default_factory=lambda: [30.0] * 12)
    kd: list = dataclasses.field(default_factory=lambda: [1.0] * 12)
    tau_ff: list = dataclasses.field(default_factory=lambda: [0.0] * 12)

    def set_standing_pose(self, hip_x=0.0, hip_y=-0.65, knee=1.30):
        """Fill with a fixed standing pose for Lite3 (12 joints, 4 legs × 3 DOF).

        Joint order: FL_HipX, FL_HipY, FL_Knee, FR_HipX, FR_HipY, FR_Knee,
                      HL_HipX, HL_HipY, HL_Knee, HR_HipX, HR_HipY, HR_Knee
        """
        # FL  FR  HL  HR
        poses = [
            hip_x,  hip_y,  knee,   # FL
           -hip_x,  hip_y,  knee,   # FR (mirror hip_x)
            hip_x,  hip_y,  knee,   # HL
           -hip_x,  hip_y,  knee,   # HR (mirror hip_x)
        ]
        self.joint_pos_des = list(poses)
        self.joint_vel_des = [0.0] * 12
        self.kp = [30.0] * 12
        self.kd = [1.0] * 12
        self.tau_ff = [0.0] * 12

    def set_gains(self, kp=30.0, kd=1.0):
        """Set uniform PD gains for all joints."""
        self.kp = [kp] * 12
        self.kd = [kd] * 12


# ============================================================================
# NxBridge
# ============================================================================

# Callback type aliases
ObservationCallback = Callable[[NxRobotState], None]
ActionCallback = Callable[[], NxJointCommand]
PolicyCallback = Callable[[NxRobotState], NxJointCommand]


class NxBridge:
    """UDP bridge to RK3588 for Lite3 robot control.

    Manages two background threads:
      - recv: receives SensorPackets, calls observation callback
      - send: periodically calls action callback, sends CommandPackets
    """

    def __init__(self,
                 recv_port: int = DEFAULT_SENSOR_PORT,
                 robot_ip: str = "192.168.1.100",
                 send_port: int = DEFAULT_COMMAND_PORT):
        self._recv_port = recv_port
        self._robot_ip = robot_ip
        self._send_port = send_port
        self._send_addr = (robot_ip, send_port)

        self._sock_recv: Optional[socket.socket] = None
        self._sock_send: Optional[socket.socket] = None

        self._running = threading.Event()
        self._recv_thread: Optional[threading.Thread] = None
        self._send_thread: Optional[threading.Thread] = None

        # Callbacks
        self._on_obs: Optional[ObservationCallback] = None
        self._get_act: Optional[ActionCallback] = None

        # For single-callback mode
        self._single_cb_mode = False
        self._latest_obs: Optional[NxRobotState] = None
        self._obs_lock = threading.Lock()

        # Sequence
        self._seq = 0
        self._seq_lock = threading.Lock()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self,
              obs_or_policy: Union[ObservationCallback, PolicyCallback],
              act: Optional[ActionCallback] = None,
              period_ms: float = 20.0):
        """Start the bridge.

        Two calling conventions:

        1. Dual-callback (obs + act separate):
           bridge.start(on_observation, get_action, period_ms=20)

        2. Single-callback (obs → act):
           bridge.start(policy_function, period_ms=20)

        Args:
            obs_or_policy: ObservationCallback (mode 1) or PolicyCallback (mode 2)
            act: ActionCallback (mode 1 only)
            period_ms: Send interval in milliseconds (default 20 → 50 Hz)
        """
        if self._running.is_set():
            return

        self._period_ms = period_ms

        if act is not None:
            # Mode 1: dual-callback
            self._single_cb_mode = False
            self._on_obs = obs_or_policy
            self._get_act = act
        else:
            # Mode 2: single-callback — obs arrives → store, send thread calls policy
            self._single_cb_mode = True
            self._on_obs = self._store_obs
            policy: PolicyCallback = obs_or_policy
            self._get_act = lambda: self._call_policy(policy)

        self._setup_sockets()
        self._running.set()
        self._seq = 0

        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self._recv_thread.start()
        self._send_thread.start()

        print(f"[NxBridge] Started — listening on :{self._recv_port}, "
              f"sending to {self._robot_ip}:{self._send_port} "
              f"at {1000.0 / period_ms:.0f} Hz")

    def stop(self):
        """Stop all threads and close sockets."""
        self._running.clear()
        if self._recv_thread and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=1.0)
        if self._send_thread and self._send_thread.is_alive():
            self._send_thread.join(timeout=1.0)
        self._close_sockets()
        print("[NxBridge] Stopped")

    def send_shutdown(self):
        """Send an explicit shutdown signal to RK3588, triggering JointDamping."""
        if self._sock_send is None:
            return
        pkt = struct.pack(_COMMAND_FMT,
                          SHUTDOWN_SEQUENCE, HEARTBEAT_SHUTDOWN,
                          *([0.0] * 60))
        self._sock_send.sendto(pkt, self._send_addr)
        print("[NxBridge] Shutdown sent to robot")

    # ------------------------------------------------------------------
    # Internal — socket setup
    # ------------------------------------------------------------------

    def _setup_sockets(self):
        # Recv socket — bind to local port to receive SensorPackets
        self._sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock_recv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock_recv.settimeout(0.1)  # 100 ms timeout for clean shutdown
        self._sock_recv.bind(("", self._recv_port))

        # Send socket — sends CommandPackets to RK3588
        self._sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _close_sockets(self):
        for sock in (self._sock_recv, self._sock_send):
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
        self._sock_recv = None
        self._sock_send = None

    # ------------------------------------------------------------------
    # Internal — single-callback helpers
    # ------------------------------------------------------------------

    def _store_obs(self, state: NxRobotState):
        with self._obs_lock:
            self._latest_obs = state

    def _call_policy(self, policy: PolicyCallback) -> NxJointCommand:
        with self._obs_lock:
            obs = self._latest_obs
        if obs is not None:
            return policy(obs)
        return NxJointCommand()  # no observation yet — zeros (RK3588 holds safe)

    # ------------------------------------------------------------------
    # Internal — recv thread
    # ------------------------------------------------------------------

    def _recv_loop(self):
        """Receive SensorPackets from RK3588, call on_obs."""
        print(f"[NxBridge] Recv thread listening on port {self._recv_port}")
        while self._running.is_set():
            try:
                data, addr = self._sock_recv.recvfrom(_SENSOR_SIZE)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(data) != _SENSOR_SIZE:
                print(f"[NxBridge] Received {len(data)} bytes, "
                      f"expected {_SENSOR_SIZE} — ignoring")
                continue

            state = self._unpack_sensor(data)
            if self._on_obs:
                self._on_obs(state)

        print("[NxBridge] Recv thread stopped")

    # ------------------------------------------------------------------
    # Internal — send thread
    # ------------------------------------------------------------------

    def _send_loop(self):
        """Periodically call get_act, serialise, send to RK3588."""
        period_s = self._period_ms / 1000.0
        hz = 1000.0 / self._period_ms
        print(f"[NxBridge] Send thread running at {hz:.0f} Hz")

        while self._running.is_set():
            tick_start = time.monotonic()

            if self._get_act:
                cmd = self._get_act()
            else:
                cmd = NxJointCommand()

            # Serialise with incrementing sequence number
            floats = (list(cmd.joint_pos_des) +
                      list(cmd.joint_vel_des) +
                      list(cmd.kp) +
                      list(cmd.kd) +
                      list(cmd.tau_ff))
            with self._seq_lock:
                seq = self._seq
                self._seq += 1
            pkt = struct.pack(_COMMAND_FMT, seq, seq, *floats)

            try:
                self._sock_send.sendto(pkt, self._send_addr)
            except OSError as e:
                print(f"[NxBridge] Send error: {e}")

            # Sleep for the remainder of the period
            elapsed = time.monotonic() - tick_start
            remaining = period_s - elapsed
            if remaining > 0:
                time.sleep(remaining)

        print("[NxBridge] Send thread stopped")

    # ------------------------------------------------------------------
    # Serialisation
    # ------------------------------------------------------------------

    @staticmethod
    def _unpack_sensor(data: bytes) -> NxRobotState:
        """Deserialise a SensorPacket from binary."""
        fields = struct.unpack(_SENSOR_FMT, data)
        seq, ts_ms, cur_state = fields[0], fields[1], fields[2]
        floats = fields[3:]

        return NxRobotState(
            seq=seq,
            timestamp_ms=ts_ms,
            current_state=cur_state,
            rpy=floats[0:3],
            acc=floats[3:6],
            omega=floats[6:9],
            joint_pos=floats[9:21],
            joint_vel=floats[21:33],
            joint_tau=floats[33:45],
            cmd_vel=floats[45:48],
        )
