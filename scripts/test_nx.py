#!/usr/bin/env python3
"""
NX-side test script for Lite3_lowlevel_sdk — handshake + standing pose.

Protocol:
  1. Wait for SensorPacket with current_state == RLHandshakeMode (5)
  2. Reply with CommandPacket(heartbeat=HEARTBEAT_READY)
  3. Wait for SensorPacket with current_state == RLControlMode (6)
  4. Enter control loop: send standing-pose CommandPacket at 50 Hz

Usage (on NX):
    python3 test_nx.py [rk3588_ip] [nx_ip]

Defaults: rk3588=192.168.1.120, nx=192.168.1.103
"""

import socket
import struct
import time
import threading
import sys
import math

# ============================================================================
# Config
# ============================================================================

RK3588_IP  = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.120"
NX_IP      = sys.argv[2] if len(sys.argv) > 2 else "192.168.1.103"
SENSOR_PORT = 31001   # RX from RK3588
CMD_PORT    = 31002   # TX to RK3588
FREQ_HZ     = 50.0

# Wire format
SENSOR_FMT = "!IIi48f"
CMD_FMT    = "!II60f"
SENSOR_SZ  = struct.calcsize(SENSOR_FMT)
CMD_SZ     = struct.calcsize(CMD_FMT)

# State constants (mirrors custom_types.h)
STATE_RL_HANDSHAKE = 5
STATE_RL_CONTROL   = 6
HEARTBEAT_READY    = 0x0B0B0B0B

# Standing pose
HIP_X, HIP_Y, KNEE = 0.0, -0.65, 1.30
KP, KD = 30.0, 1.0

# ============================================================================
# Helpers
# ============================================================================

def build_cmd_packet(seq, heartbeat, pos_des, kp=KP, kd=KD):
    """Return packed CommandPacket bytes."""
    vel = [0.0] * 12
    kp_arr  = [kp] * 12
    kd_arr  = [kd] * 12
    tau = [0.0] * 12
    return struct.pack(CMD_FMT, seq, heartbeat, *(pos_des + vel + kp_arr + kd_arr + tau))

def build_standing_pose():
    return [HIP_X, HIP_Y, KNEE,           # FL
            -HIP_X, HIP_Y, KNEE,          # FR
            HIP_X, HIP_Y, KNEE,           # HL
            -HIP_X, HIP_Y, KNEE]          # HR

def unpack_sensor(data):
    f = struct.unpack(SENSOR_FMT, data)
    return {
        "seq": f[0], "ts": f[1], "state": f[2],
        "rpy": f[3:6], "acc": f[6:9], "omega": f[9:12],
        "jpos": f[12:24], "jvel": f[24:36], "jtau": f[36:48],
        "cmd_vel": f[48:51],
    }

# ============================================================================
# Main
# ============================================================================

def main():
    print(f"=== Lite3 NX Test (Handshake) ===")
    print(f"RX sensor : {NX_IP}:{SENSOR_PORT}")
    print(f"TX command: {RK3588_IP}:{CMD_PORT}")
    print()

    # Setup sockets
    sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock_recv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock_recv.settimeout(0.5)
    sock_recv.bind(("", SENSOR_PORT))  # INADDR_ANY — listen on all interfaces

    sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rk_addr = (RK3588_IP, CMD_PORT)
    standing_pose = build_standing_pose()

    print(f"Recv: bound to :{SENSOR_PORT} (any interface)")
    print(f"Send: → {RK3588_IP}:{CMD_PORT}")

    # ================================================================
    # Phase 1: Wait for handshake request from RK3588
    # ================================================================
    print()
    print("[HANDSHAKE] Waiting for RLHandshakeMode (state=5) from RK3588...")
    print("            (Make sure rl_deploy is running and you pressed")
    print("             the RL control button on the gamepad)")
    print()

    while True:
        try:
            data, addr = sock_recv.recvfrom(SENSOR_SZ)
        except socket.timeout:
            continue
        if len(data) != SENSOR_SZ:
            print(f"[!] Received {len(data)} bytes from {addr}, expected {SENSOR_SZ}")
            continue
        s = unpack_sensor(data)
        if s["state"] == STATE_RL_HANDSHAKE:
            print(f"[HANDSHAKE] Received RLHandshake from {addr} (seq={s['seq']}) — sending READY")
            break
        # Diagnostic: received a packet but wrong state
        if s["seq"] % 100 == 0:
            print(f"[DIAG] Received state={s['state']} from {addr}, waiting for state={STATE_RL_HANDSHAKE}")

    # Reply READY (send a few times to ensure delivery)
    for _ in range(3):
        pkt = build_cmd_packet(0, HEARTBEAT_READY, standing_pose)
        sock_send.sendto(pkt, rk_addr)
        time.sleep(0.01)

    # ================================================================
    # Phase 2: Wait for RLControl mode
    # ================================================================
    print("[HANDSHAKE] Waiting for RLControlMode...")

    while True:
        try:
            data, _ = sock_recv.recvfrom(SENSOR_SZ)
        except socket.timeout:
            continue
        if len(data) != SENSOR_SZ:
            continue
        s = unpack_sensor(data)
        if s["state"] == STATE_RL_CONTROL:
            print(f"[HANDSHAKE] RLControl confirmed (seq={s['seq']}) — starting control loop")
            break

    # ================================================================
    # Phase 3: Control loop — standing pose at 50 Hz
    # ================================================================
    seq = 0
    last_print = 0.0
    pkt_count = 0
    period = 1.0 / FREQ_HZ
    running = True

    print(f"[CONTROL] Running at {FREQ_HZ:.0f} Hz → {RK3588_IP}:{CMD_PORT}")
    print("          Press Ctrl+C to stop")

    def send_thread():
        nonlocal seq
        while running:
            t0 = time.monotonic()
            pkt = build_cmd_packet(seq, seq, standing_pose)
            seq += 1
            sock_send.sendto(pkt, rk_addr)
            elapsed = time.monotonic() - t0
            if (remain := period - elapsed) > 0:
                time.sleep(remain)

    send_th = threading.Thread(target=send_thread, daemon=True)
    send_th.start()

    try:
        while True:
            try:
                data, _ = sock_recv.recvfrom(SENSOR_SZ)
            except socket.timeout:
                continue
            if len(data) != SENSOR_SZ:
                continue

            pkt_count += 1
            s = unpack_sensor(data)
            now = time.time()
            if now - last_print >= 1.0:
                last_print = now
                rpy_deg = [math.degrees(v) for v in s["rpy"]]
                print(f"[{pkt_count:5d}] seq={s['seq']} state={s['state']} "
                      f"rpy=({rpy_deg[0]:5.1f},{rpy_deg[1]:5.1f},{rpy_deg[2]:5.1f}) deg  "
                      f"cmd=({s['cmd_vel'][0]:.2f},{s['cmd_vel'][1]:.2f},{s['cmd_vel'][2]:.2f})  "
                      f"q[0]={s['jpos'][0]:.2f} q[1]={s['jpos'][1]:.2f} q[2]={s['jpos'][2]:.2f}")

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        running = False
        send_th.join(timeout=1)
        sock_recv.close()
        sock_send.close()
        print("Done.")

if __name__ == "__main__":
    main()
