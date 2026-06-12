/**
 * @file nx_bridge.cpp
 * @brief Implementation of the NX-side communication bridge
 */

#include "nx_bridge_api.hpp"
#include "nx_network_codes.hpp"

#include <iostream>
#include <cstring>
#include <thread>
#include <mutex>
#include <atomic>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

// ============================================================================
// File-scope configuration — safe default PD gains
// ============================================================================
// These are the fallback gains used when the action callback returns zeros.
// They are set via NxBridge::SetDefaultGains().

static float g_default_kp = 30.0f;
static float g_default_kd = 1.0f;

// ============================================================================
// PIMPL — hides all socket and threading details from the public header
// ============================================================================

struct NxBridge::Impl {
    std::string robot_ip;
    int send_port, recv_port;

    int sock_recv = -1;
    int sock_send = -1;
    struct sockaddr_in robot_addr;

    // Thread state
    std::thread recv_thread, send_thread;
    std::atomic<bool> running{false};

    // Callbacks
    ObservationCallback on_obs;
    ActionCallback      get_act;

    // For the single-callback overload: stash the latest observation
    std::mutex latest_obs_mutex;
    NxRobotState latest_obs;
    bool has_obs = false;
    bool single_cb_mode = false;

    // Send rate
    float send_period_ms = 20.0f;

    // Sequence
    uint32_t seq = 0;

    // ========================================================================
    // Recv thread — blocks on SensorPacket, deserialises, calls callback
    // ========================================================================

    void RecvThread() {
        SensorPacket pkt;
        struct sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);

        std::cout << "[NxBridge] Recv thread listening on port " << recv_port << std::endl;

        while (running.load()) {
            std::memset(&pkt, 0, sizeof(pkt));
            ssize_t n = recvfrom(sock_recv, &pkt, sizeof(pkt), 0,
                                 (struct sockaddr*)&from_addr, &from_len);

            if (n < 0) continue;  // timeout or error — loop

            if (static_cast<size_t>(n) != sizeof(SensorPacket)) {
                std::cerr << "[NxBridge] Received " << n
                          << " bytes, expected " << sizeof(SensorPacket) << std::endl;
                continue;
            }

            // Network-byte-order → host for integer fields
            pkt.seq           = ntohl(pkt.seq);
            pkt.timestamp_ms  = ntohl(pkt.timestamp_ms);
            pkt.current_state = static_cast<int32_t>(ntohl(
                static_cast<uint32_t>(pkt.current_state)));

            // Copy to the public-facing struct
            NxRobotState state;
            state.seq           = pkt.seq;
            state.timestamp_ms  = pkt.timestamp_ms;
            state.current_state = pkt.current_state;
            std::memcpy(state.rpy,       pkt.rpy,       sizeof(state.rpy));
            std::memcpy(state.acc,       pkt.acc,       sizeof(state.acc));
            std::memcpy(state.omega,     pkt.omega,     sizeof(state.omega));
            std::memcpy(state.joint_pos, pkt.joint_pos, sizeof(state.joint_pos));
            std::memcpy(state.joint_vel, pkt.joint_vel, sizeof(state.joint_vel));
            std::memcpy(state.joint_tau, pkt.joint_tau, sizeof(state.joint_tau));
            std::memcpy(state.cmd_vel,   pkt.cmd_vel,   sizeof(state.cmd_vel));

            if (single_cb_mode) {
                // Store the latest observation; the send thread's get_act
                // will pick it up on the next timer tick.
                std::lock_guard<std::mutex> lock(latest_obs_mutex);
                latest_obs = state;
                has_obs = true;
            } else if (on_obs) {
                on_obs(state);
            }
        }
        std::cout << "[NxBridge] Recv thread stopped" << std::endl;
    }

    // ========================================================================
    // Send thread — periodically calls get_act, serialises, sends to RK3588
    // ========================================================================

    void SendThread() {
        long period_ns = static_cast<long>(send_period_ms * 1000000.0);

        // timerfd + epoll at the configured rate (default 50 Hz)
        int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
        if (tfd < 0) {
            std::cerr << "[NxBridge] timerfd_create failed" << std::endl;
            return;
        }

        struct itimerspec time_intv;
        time_intv.it_value.tv_sec  = 0;
        time_intv.it_value.tv_nsec = period_ns;
        time_intv.it_interval.tv_sec  = 0;
        time_intv.it_interval.tv_nsec = period_ns;
        timerfd_settime(tfd, 0, &time_intv, nullptr);

        int efd = epoll_create1(0);
        if (efd < 0) { close(tfd); return; }

        struct epoll_event ev;
        ev.data.fd = tfd;
        ev.events  = EPOLLIN;
        epoll_ctl(efd, EPOLL_CTL_ADD, tfd, &ev);

        struct epoll_event events[1];
        std::cout << "[NxBridge] Send thread running at "
                  << (1000.0 / send_period_ms) << " Hz" << std::endl;

        while (running.load()) {
            int n = epoll_wait(efd, events, 1, -1);
            if (n < 0) continue;

            uint64_t val;
            read(tfd, &val, sizeof(val));  // consume timer event

            // Obtain the next command from the developer's callback
            NxJointCommand cmd;
            std::memset(&cmd, 0, sizeof(cmd));
            if (get_act) {
                cmd = get_act();
            }

            // Apply safe defaults for any zero gains
            for (int i = 0; i < 12; ++i) {
                if (cmd.kp[i] <= 0.0f) cmd.kp[i] = g_default_kp;
                if (cmd.kd[i] <= 0.0f) cmd.kd[i] = g_default_kd;
            }

            // Serialise and send
            CommandPacket pkt;
            std::memset(&pkt, 0, sizeof(pkt));
            pkt.seq       = htonl(seq++);
            pkt.heartbeat = htonl(seq);
            std::memcpy(pkt.joint_pos_des, cmd.joint_pos_des, sizeof(pkt.joint_pos_des));
            std::memcpy(pkt.joint_vel_des, cmd.joint_vel_des, sizeof(pkt.joint_vel_des));
            std::memcpy(pkt.kp,             cmd.kp,             sizeof(pkt.kp));
            std::memcpy(pkt.kd,             cmd.kd,             sizeof(pkt.kd));
            std::memcpy(pkt.tau_ff,         cmd.tau_ff,         sizeof(pkt.tau_ff));

            sendto(sock_send, &pkt, sizeof(pkt), 0,
                   (struct sockaddr*)&robot_addr, sizeof(robot_addr));
        }

        close(efd);
        close(tfd);
        std::cout << "[NxBridge] Send thread stopped" << std::endl;
    }
};

// ============================================================================
// Public API
// ============================================================================

NxBridge::NxBridge(int recv_port, const std::string& robot_ip, int send_port)
    : impl_(std::make_unique<Impl>())
{
    impl_->robot_ip  = robot_ip;
    impl_->send_port = send_port;
    impl_->recv_port = recv_port;
}

NxBridge::~NxBridge() {
    Stop();
}

void NxBridge::Start(ObservationCallback on_obs, ActionCallback get_act, float period_ms) {
    if (impl_->running.load()) return;

    impl_->on_obs = std::move(on_obs);
    impl_->get_act = std::move(get_act);
    impl_->send_period_ms = period_ms;
    impl_->single_cb_mode = false;

    // --- Create recv socket ---
    impl_->sock_recv = socket(AF_INET, SOCK_DGRAM, 0);
    if (impl_->sock_recv < 0) {
        std::cerr << "[NxBridge] Failed to create recv socket" << std::endl;
        return;
    }
    int reuse = 1;
    setsockopt(impl_->sock_recv, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    struct timeval tv;
    tv.tv_sec  = 0;
    tv.tv_usec = 100000;  // 100 ms timeout for clean thread shutdown
    setsockopt(impl_->sock_recv, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    struct sockaddr_in recv_addr;
    std::memset(&recv_addr, 0, sizeof(recv_addr));
    recv_addr.sin_family      = AF_INET;
    recv_addr.sin_port        = htons(impl_->recv_port);
    recv_addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(impl_->sock_recv, (struct sockaddr*)&recv_addr, sizeof(recv_addr)) < 0) {
        std::cerr << "[NxBridge] Failed to bind recv socket to port "
                  << impl_->recv_port << std::endl;
        close(impl_->sock_recv);
        impl_->sock_recv = -1;
        return;
    }

    // --- Create send socket ---
    impl_->sock_send = socket(AF_INET, SOCK_DGRAM, 0);
    if (impl_->sock_send < 0) {
        std::cerr << "[NxBridge] Failed to create send socket" << std::endl;
        close(impl_->sock_recv);
        impl_->sock_recv = -1;
        return;
    }
    std::memset(&impl_->robot_addr, 0, sizeof(impl_->robot_addr));
    impl_->robot_addr.sin_family = AF_INET;
    impl_->robot_addr.sin_port   = htons(impl_->send_port);
    impl_->robot_addr.sin_addr.s_addr = inet_addr(impl_->robot_ip.c_str());

    // --- Launch threads ---
    impl_->running.store(true);
    impl_->seq = 0;
    impl_->recv_thread = std::thread(&Impl::RecvThread, impl_.get());
    impl_->send_thread = std::thread(&Impl::SendThread, impl_.get());

    std::cout << "[NxBridge] Started — listening on :" << impl_->recv_port
              << ", sending to " << impl_->robot_ip << ":" << impl_->send_port
              << " at " << (1000.0f / period_ms) << " Hz" << std::endl;
}

void NxBridge::Start(std::function<NxJointCommand(const NxRobotState&)> policy, float period_ms) {
    // Single-callback mode: recv stores latest obs (thread-safe),
    // send calls policy with that obs.

    Impl* impl = impl_.get();
    impl->single_cb_mode = true;

    ObservationCallback store_obs = [impl](const NxRobotState& state) {
        std::lock_guard<std::mutex> lock(impl->latest_obs_mutex);
        impl->latest_obs = state;
        impl->has_obs = true;
    };

    ActionCallback wrapped_get_act = [impl, policy = std::move(policy)]() -> NxJointCommand {
        NxRobotState obs;
        bool valid = false;
        {
            std::lock_guard<std::mutex> lock(impl->latest_obs_mutex);
            if (impl->has_obs) {
                obs = impl->latest_obs;
                valid = true;
            }
        }
        if (valid) {
            return policy(obs);
        }
        // No observation yet — return zeros (RK3588 holds current position safely)
        NxJointCommand cmd;
        std::memset(&cmd, 0, sizeof(cmd));
        return cmd;
    };

    Start(std::move(store_obs), std::move(wrapped_get_act), period_ms);
}

void NxBridge::Stop() {
    impl_->running.store(false);
    if (impl_->recv_thread.joinable()) impl_->recv_thread.join();
    if (impl_->send_thread.joinable()) impl_->send_thread.join();
    if (impl_->sock_recv >= 0) { close(impl_->sock_recv); impl_->sock_recv = -1; }
    if (impl_->sock_send >= 0) { close(impl_->sock_send); impl_->sock_send = -1; }
}

void NxBridge::SendShutdown() {
    if (impl_->sock_send < 0) return;

    CommandPacket pkt;
    std::memset(&pkt, 0, sizeof(pkt));
    pkt.seq       = htonl(SHUTDOWN_SEQUENCE);
    pkt.heartbeat = htonl(HEARTBEAT_SHUTDOWN);

    sendto(impl_->sock_send, &pkt, sizeof(pkt), 0,
           (struct sockaddr*)&impl_->robot_addr, sizeof(impl_->robot_addr));

    std::cout << "[NxBridge] Shutdown sent to robot" << std::endl;
}

void NxBridge::SetDefaultGains(float kp, float kd) {
    g_default_kp = kp;
    g_default_kd = kd;
}
