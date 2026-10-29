#pragma once

// Cemuhook DSU ("DualShock UDP") client protocol: the messages a client sends, the pad data a server answers with,
// and the conversion of its motion to Nebula's sensor frame. DSU servers such as SC2DSU (Steam Controllers), BetterJoy,
// DS4Windows and Joy2Win serve a controller's motion on UDP 127.0.0.1:26760 by default.
//
// Layout (all little-endian): a 16-byte header {magic "DSUC" client / "DSUS" server, u16 protocol version 1001,
// u16 length after the header, u32 CRC-32 of the whole packet with this field zeroed, u32 sender id}, then a u32
// message type. Pad data (0x100002) continues with the 11-byte slot header {slot, state, model, connection, MAC[6],
// battery}, an active flag, a u32 packet number, buttons, sticks, analog buttons, two touches, a u64 motion timestamp
// in microseconds, accelerometer x/y/z in g and gyroscope pitch/yaw/roll in degrees per second (floats): 100 bytes.
//
// Axes. As Dolphin, Cemu and Eden/Yuzu read them (and SC2DSU writes them): the accelerometer reads -1 g on Y when the
// controller lies flat (the negated reaction to gravity, X right, Y up, Z toward the player), pitch is positive nose
// up, yaw positive turning right, roll positive rolling right. Nebula's frame (switch_controller.h ImuSample, SDL's)
// reads +1 g up and turns counter-clockwise about each axis, so accel = -a and gyro = (pitch, -yaw, -roll) in rad/s.
// DsuAxisFlips turns any of them around for a server that sends otherwise.

#include "galaxy/switch_controller.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>

namespace galaxy::input::dsu {

inline constexpr std::uint16_t kProtocolVersion = 1001;
inline constexpr std::uint16_t kDefaultPort = 26760;
inline constexpr std::size_t kHeaderBytes = 16;
inline constexpr std::size_t kSlotCount = 4;
inline constexpr std::uint32_t kMessageVersion = 0x100000u;
inline constexpr std::uint32_t kMessagePorts = 0x100001u;
inline constexpr std::uint32_t kMessagePadData = 0x100002u;
inline constexpr std::size_t kPadDataBytes = 100;
inline constexpr std::size_t kPortInfoBytes = 32;
// A server forgets a subscription it has not heard of for 5 s; clients renew well inside that.
inline constexpr std::uint64_t kRenewIntervalUs = 1'000'000;

// CRC-32 (IEEE 802.3, reflected 0xEDB88320) as the protocol uses it.
[[nodiscard]] std::uint32_t crc32(std::span<const std::uint8_t> data) noexcept;

// Client requests, finished with the CRC. Returns the bytes written (0 when `out` is too small).
// Ports: the slots asked about (at most kSlotCount).
std::size_t build_version_request(std::uint32_t client_id, std::span<std::uint8_t> out) noexcept;
std::size_t build_ports_request(std::uint32_t client_id, std::span<const std::uint8_t> slots,
                                std::span<std::uint8_t> out) noexcept;
// Pad data for one slot (registration by slot id); `all_slots` subscribes to every slot instead.
std::size_t build_pad_data_request(std::uint32_t client_id, std::uint8_t slot, bool all_slots,
                                   std::span<std::uint8_t> out) noexcept;

// A server packet's header checked: magic "DSUS", a protocol version, a length that fits, the CRC. Returns the
// message type.
[[nodiscard]] std::optional<std::uint32_t> validate_server_packet(std::span<const std::uint8_t> packet) noexcept;

struct PadData {
    std::uint8_t slot = 0;
    std::uint8_t slot_state = 0;  // 2: connected
    std::uint8_t model = 0;       // 2: full gyro
    bool active = false;
    std::uint32_t packet_number = 0;
    std::uint64_t motion_timestamp_us = 0;
    std::array<float, 3> accel_g{};     // as sent
    std::array<float, 3> gyro_dps{};    // pitch, yaw, roll as sent
};
// A validated pad data packet (validate_server_packet first, or not: this checks too).
[[nodiscard]] std::optional<PadData> parse_pad_data(std::span<const std::uint8_t> packet) noexcept;

struct DsuAxisFlips {
    bool pitch = false;
    bool yaw = false;
    bool roll = false;
    bool accel = false;  // the server sends the reaction (+1 g up at rest) instead
};
// The sample in Nebula's frame (ImuSample: g and rad/s).
[[nodiscard]] switch_hid::ImuSample to_imu_sample(const PadData& pad, const DsuAxisFlips& flips = {}) noexcept;

// Seconds from the previous motion sample: the server's motion clock when it advanced by a believable amount
// (0.5 ms to 50 ms, across a 32-bit wrap too), else `host_dt` clamped to 50 ms.
[[nodiscard]] float sample_dt(std::uint64_t previous_us, std::uint64_t now_us, float host_dt) noexcept;

// The client's settings, dsu.ini in Nebula's data folder (key=value, '#' or ';' comments).
enum class ShakeTarget : std::uint8_t { Off, Remote, Nunchuk };
struct DsuConfig {
    bool enabled = true;
    std::string server = "127.0.0.1";
    std::uint16_t port = kDefaultPort;
    // The server slot whose motion is used; -1: the first slot that sends motion.
    int slot = -1;
    bool gyro_pointer = true;
    float gyro_sensitivity = 1.0f;  // 1: a picture width per 40 degrees of turn (0.2 .. 5)
    ShakeTarget shake = ShakeTarget::Remote;
    std::uint8_t shake_sensitivity = 3;  // 1 (strong shake) .. 5 (light)
    bool tilt = true;  // the tilt sections steer by the controller's tilt
    DsuAxisFlips flips{};
    friend bool operator==(const DsuConfig& a, const DsuConfig& b) {
        return a.enabled == b.enabled && a.server == b.server && a.port == b.port && a.slot == b.slot &&
               a.gyro_pointer == b.gyro_pointer && a.gyro_sensitivity == b.gyro_sensitivity &&
               a.shake == b.shake && a.shake_sensitivity == b.shake_sensitivity && a.tilt == b.tilt &&
               a.flips.pitch == b.flips.pitch && a.flips.yaw == b.flips.yaw && a.flips.roll == b.flips.roll &&
               a.flips.accel == b.flips.accel;
    }
};
// Unknown keys and bad values keep the defaults.
[[nodiscard]] DsuConfig parse_config(std::string_view text) noexcept;
// The commented file written when none exists.
[[nodiscard]] std::string default_config_text();
// "host", "host:port" or ":port" over `config` (the NEBULA_DSU_SERVER override).
void apply_server_override(std::string_view value, DsuConfig& config) noexcept;

}  // namespace galaxy::input::dsu
