#include "galaxy/dsu_client.h"

#include "galaxy/content_viewport.h"
#include "galaxy/controller_hid.h"
#include "galaxy/platform_paths.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <WinSock2.h>
#include <WS2tcpip.h>
#include <mstcpip.h>
#include <Windows.h>
#ifndef SIO_UDP_CONNRESET
#define SIO_UDP_CONNRESET _WSAIOW(IOC_VENDOR, 12)
#endif

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

namespace galaxy::host {

namespace {

namespace dsu = galaxy::input::dsu;
namespace sw = galaxy::input::switch_hid;

constexpr std::uint64_t kLiveWindowUs = 500'000;
constexpr std::uint64_t kSlotLockLostUs = 1'000'000;
constexpr std::uint64_t kConfigCheckUs = 1'000'000;
// Pointer movement nobody read for this long (the keyboard in use, a menu that polls no controller) is dropped, so
// the pointer does not jump by everything turned meanwhile when the controller is read again.
constexpr std::uint64_t kStaleAccumulationUs = 250'000;
constexpr DWORD kReceiveTimeoutMs = 100;

struct Shared {
    SRWLOCK lock = SRWLOCK_INIT;
    std::atomic<bool> started{false};
    std::atomic<bool> enabled{false};
    std::mutex lifecycle;
    DsuMotionState state{};
    std::uint64_t last_sample_us = 0;
    std::uint64_t last_consume_us = 0;
};

Shared& shared() {
    static Shared s;
    return s;
}

std::string env(const char* name) {
    char value[256]{};
    const DWORD length = GetEnvironmentVariableA(name, value, sizeof(value));
    return length == 0u || length >= sizeof(value) ? std::string{} : std::string(value, length);
}

bool env_off(const char* name) {
    const std::string v = env(name);
    return v == "0" || v == "off" || v == "OFF" || v == "false" || v == "no";
}

std::filesystem::path config_path() {
    const std::filesystem::path folder = galaxy::nebula_data_directory();
    return folder.empty() ? std::filesystem::path{} : folder / L"dsu.ini";
}

// The settings file as it is now (written with the defaults when missing), with the environment's overrides.
dsu::DsuConfig load_config(std::filesystem::file_time_type& stamp) {
    dsu::DsuConfig config{};
    const std::filesystem::path path = config_path();
    if (!path.empty()) {
        std::error_code error;
        if (!std::filesystem::exists(path, error)) {
            std::filesystem::create_directories(path.parent_path(), error);
            std::ofstream out(path, std::ios::binary);
            if (out) out << dsu::default_config_text();
        }
        stamp = std::filesystem::last_write_time(path, error);
        std::ifstream in(path, std::ios::binary);
        if (in) {
            std::ostringstream text;
            text << in.rdbuf();
            config = dsu::parse_config(text.str());
        }
    }
    if (env_off("NEBULA_DSU")) config.enabled = false;
    if (const std::string server = env("NEBULA_DSU_SERVER"); !server.empty()) {
        dsu::apply_server_override(server, config);
    }
    return config;
}

bool config_changed(const std::filesystem::file_time_type& stamp) {
    const std::filesystem::path path = config_path();
    if (path.empty()) return false;
    std::error_code error;
    const auto now = std::filesystem::last_write_time(path, error);
    return !error && now != stamp;
}

class Worker {
public:
    explicit Worker(dsu::DsuConfig config, std::filesystem::file_time_type stamp)
        : config_(std::move(config)), stamp_(stamp) {}

    void run() {
        WSADATA wsa{};
        if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) {
            std::fprintf(stderr, "[dsu] Winsock did not start; DSU motion is off\n");
            return;
        }
        open_socket();
        std::uint64_t next_renew = 0;
        std::uint64_t next_config = steady_us() + kConfigCheckUs;
        std::array<std::uint8_t, 512> packet{};
        for (;;) {
            const std::uint64_t now = steady_us();
            if (now >= next_config) {
                next_config = now + kConfigCheckUs;
                if (config_changed(stamp_)) {
                    const dsu::DsuConfig previous = config_;
                    config_ = load_config(stamp_);
                    std::fprintf(stderr, "[dsu] settings changed: %s %s:%u slot=%d\n",
                                 config_.enabled ? "on" : "off", config_.server.c_str(),
                                 static_cast<unsigned>(config_.port), config_.slot);
                    shared().enabled.store(config_.enabled, std::memory_order_release);
                    if (previous.server != config_.server || previous.port != config_.port ||
                        previous.slot != config_.slot) {
                        open_socket();
                        reset_slots();
                        next_renew = 0;
                    }
                }
            }
            if (!config_.enabled || socket_ == INVALID_SOCKET) {
                Sleep(250);
                continue;
            }
            if (now >= next_renew) {
                next_renew = now + dsu::kRenewIntervalUs;
                subscribe();
            }
            sockaddr_storage from{};
            int from_length = static_cast<int>(sizeof(from));
            const int got = recvfrom(socket_, reinterpret_cast<char*>(packet.data()),
                                     static_cast<int>(packet.size()), 0, reinterpret_cast<sockaddr*>(&from),
                                     &from_length);
            if (got <= 0) continue;  // timeout, or an ICMP reset while no server listens
            handle_packet(std::span<const std::uint8_t>(packet.data(), static_cast<std::size_t>(got)));
        }
    }

private:
    struct Slot {
        sw::SwitchMotion motion{};
        std::uint32_t last_packet = 0;
        std::uint64_t last_timestamp_us = 0;
        std::uint64_t last_host_us = 0;
        bool seen = false;
    };

    static std::uint64_t steady_us() noexcept { return controller_hid_now_us(); }

    void open_socket() {
        if (socket_ != INVALID_SOCKET) {
            closesocket(socket_);
            socket_ = INVALID_SOCKET;
        }
        server_length_ = 0;
        addrinfo hints{};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;
        hints.ai_protocol = IPPROTO_UDP;
        addrinfo* found = nullptr;
        const std::string port = std::to_string(config_.port);
        if (getaddrinfo(config_.server.c_str(), port.c_str(), &hints, &found) != 0 || found == nullptr) {
            std::fprintf(stderr, "[dsu] cannot resolve %s; DSU motion waits for a valid server\n",
                         config_.server.c_str());
            return;
        }
        SOCKET s = socket(found->ai_family, SOCK_DGRAM, IPPROTO_UDP);
        if (s == INVALID_SOCKET) {
            freeaddrinfo(found);
            return;
        }
        std::memcpy(&server_, found->ai_addr, found->ai_addrlen);
        server_length_ = static_cast<int>(found->ai_addrlen);
        freeaddrinfo(found);
        // Without a server, Windows reports the ICMP port-unreachable as WSAECONNRESET on the next receive; it
        // means nothing to a client that keeps asking.
        BOOL report_reset = FALSE;
        DWORD returned = 0;
        WSAIoctl(s, SIO_UDP_CONNRESET, &report_reset, static_cast<DWORD>(sizeof(report_reset)), nullptr, 0, &returned, nullptr,
                 nullptr);
        const DWORD timeout = kReceiveTimeoutMs;
        setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout),
                   static_cast<int>(sizeof(timeout)));
        socket_ = s;
        std::fprintf(stderr, "[dsu] client for %s:%u, slot %s\n", config_.server.c_str(),
                     static_cast<unsigned>(config_.port),
                     config_.slot < 0 ? "auto" : std::to_string(config_.slot).c_str());
    }

    void send(std::span<const std::uint8_t> bytes) {
        if (bytes.empty() || server_length_ == 0) return;
        sendto(socket_, reinterpret_cast<const char*>(bytes.data()), static_cast<int>(bytes.size()), 0,
               reinterpret_cast<const sockaddr*>(&server_), server_length_);
    }

    void subscribe() {
        std::array<std::uint8_t, 64> out{};
        if (!announced_) {
            send(std::span(out.data(), dsu::build_version_request(client_id_, out)));
            constexpr std::uint8_t kAll[4] = {0, 1, 2, 3};
            send(std::span(out.data(), dsu::build_ports_request(client_id_, kAll, out)));
            announced_ = true;
        }
        const bool all = config_.slot < 0;
        send(std::span(out.data(), dsu::build_pad_data_request(client_id_, static_cast<std::uint8_t>(
                                                                   std::max(config_.slot, 0)), all, out)));
    }

    void reset_slots() {
        for (Slot& slot : slots_) slot = Slot{};
        locked_slot_ = -1;
        announced_ = false;
        AcquireSRWLockExclusive(&shared().lock);
        shared().state = DsuMotionState{};
        shared().last_sample_us = 0;
        ReleaseSRWLockExclusive(&shared().lock);
    }

    void handle_packet(std::span<const std::uint8_t> packet) {
        const std::optional<dsu::PadData> pad = dsu::parse_pad_data(packet);
        if (!pad || pad->slot >= dsu::kSlotCount) return;
        if (!pad->active || pad->slot_state != 2u) return;
        const std::uint64_t now = steady_us();
        // The slot whose motion is used: the configured one, or the first that sends motion, kept until it falls
        // silent.
        if (config_.slot >= 0) {
            if (pad->slot != config_.slot) return;
        } else {
            if (locked_slot_ >= 0 && now - slots_[static_cast<std::size_t>(locked_slot_)].last_host_us >
                                         kSlotLockLostUs) {
                locked_slot_ = -1;
            }
            if (locked_slot_ < 0) {
                locked_slot_ = pad->slot;
                std::fprintf(stderr, "[dsu] motion from server slot %u\n", static_cast<unsigned>(pad->slot));
            }
            if (pad->slot != locked_slot_) return;
        }
        Slot& slot = slots_[pad->slot];
        // The same sample twice (several subscriptions, a resend) moves nothing.
        if (slot.seen && pad->packet_number == slot.last_packet) return;
        if (slot.seen && pad->motion_timestamp_us != 0u && pad->motion_timestamp_us == slot.last_timestamp_us) {
            slot.last_packet = pad->packet_number;
            return;
        }
        const float host_dt = slot.last_host_us != 0u ? static_cast<float>(now - slot.last_host_us) * 1e-6f : 0.0f;
        const float dt = dsu::sample_dt(slot.seen ? slot.last_timestamp_us : 0u, pad->motion_timestamp_us, host_dt);
        slot.seen = true;
        slot.last_packet = pad->packet_number;
        slot.last_timestamp_us = pad->motion_timestamp_us;
        slot.last_host_us = now;

        sw::MotionSettings settings{};
        settings.shake = config_.shake != dsu::ShakeTarget::Off;
        settings.shake_sensitivity = config_.shake_sensitivity;
        settings.gyro_pointer = config_.gyro_pointer;
        settings.gyro_sensitivity = config_.gyro_sensitivity;
        settings.picture_aspect = static_cast<float>(galaxy::kWiiContentAspect);
        const sw::ImuSample sample = dsu::to_imu_sample(*pad, config_.flips);
        const sw::SwitchMotion::Output out = slot.motion.feed(sample, dt, now, settings);

        AcquireSRWLockExclusive(&shared().lock);
        DsuMotionState& state = shared().state;
        if (now > shared().last_consume_us + kStaleAccumulationUs) state.gyro_x = state.gyro_y = 0.0f;
        state.server_slot = pad->slot;
        state.gyro_x = std::clamp(state.gyro_x + out.pointer_x, -4.0f, 4.0f);
        state.gyro_y = std::clamp(state.gyro_y + out.pointer_y, -4.0f, 4.0f);
        if (out.shake_started) state.shake_until_us = now + sw::SwitchMotion::kShakeHoldUs;
        state.gravity = slot.motion.gravity();
        state.gravity_valid = slot.motion.gravity_ready();
        state.gyro_pointer = config_.gyro_pointer;
        state.tilt = config_.tilt;
        state.shake = config_.shake;
        shared().last_sample_us = now;
        ReleaseSRWLockExclusive(&shared().lock);

        ++samples_;
        if (now - logged_us_ >= 10'000'000u) {
            std::fprintf(stderr, "[dsu] slot=%u samples=%llu gravity=%s\n", static_cast<unsigned>(pad->slot),
                         static_cast<unsigned long long>(samples_), state_gravity_text(slot).c_str());
            logged_us_ = now;
        }
    }

    static std::string state_gravity_text(const Slot& slot) {
        char text[64];
        const auto& g = slot.motion.gravity();
        std::snprintf(text, sizeof(text), "%.2f,%.2f,%.2f%s", g[0], g[1], g[2],
                      slot.motion.gravity_ready() ? "" : " (settling)");
        return text;
    }

    dsu::DsuConfig config_;
    std::filesystem::file_time_type stamp_;
    SOCKET socket_ = INVALID_SOCKET;
    sockaddr_storage server_{};
    int server_length_ = 0;
    std::uint32_t client_id_ = static_cast<std::uint32_t>(GetCurrentProcessId()) ^ 0x4E42554Cu;  // "NBUL"
    bool announced_ = false;
    std::array<Slot, dsu::kSlotCount> slots_{};
    int locked_slot_ = -1;
    std::uint64_t samples_ = 0;
    std::uint64_t logged_us_ = 0;
};

}  // namespace

void ensure_dsu_client_started() noexcept {
    Shared& s = shared();
    if (s.started.load(std::memory_order_acquire)) return;
    const std::lock_guard<std::mutex> lock(s.lifecycle);
    if (s.started.load(std::memory_order_relaxed)) return;
    s.started.store(true, std::memory_order_release);
    try {
        std::filesystem::file_time_type stamp{};
        dsu::DsuConfig config = load_config(stamp);
        s.enabled.store(config.enabled, std::memory_order_release);
        if (!config.enabled) {
            std::fprintf(stderr, "[dsu] off (dsu.ini enabled=0 or NEBULA_DSU=0)\n");
        }
        // The worker runs for the life of the process (it also watches dsu.ini, so turning the client on in the
        // file starts it); it blocks at most kReceiveTimeoutMs at a time.
        std::thread([config = std::move(config), stamp]() mutable {
            SetThreadDescription(GetCurrentThread(), L"nebula-dsu-client");
            Worker worker(std::move(config), stamp);
            worker.run();
        }).detach();
    } catch (...) {
    }
}

bool read_dsu_motion(DsuMotionState& out, bool consume) noexcept {
    Shared& s = shared();
    if (!s.started.load(std::memory_order_acquire) || !s.enabled.load(std::memory_order_acquire)) return false;
    const std::uint64_t now = controller_hid_now_us();
    if (consume) AcquireSRWLockExclusive(&s.lock); else AcquireSRWLockShared(&s.lock);
    out = s.state;
    out.live = s.last_sample_us != 0u && now - s.last_sample_us < kLiveWindowUs;
    if (consume) {
        s.state.gyro_x = 0.0f;
        s.state.gyro_y = 0.0f;
        s.last_consume_us = now;
    }
    if (consume) ReleaseSRWLockExclusive(&s.lock); else ReleaseSRWLockShared(&s.lock);
    if (!out.live) {
        out.gyro_x = out.gyro_y = 0.0f;
        out.gravity_valid = false;
    }
    return true;
}

}  // namespace galaxy::host
