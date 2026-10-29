#include "galaxy/dsu_protocol.h"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstring>

namespace galaxy::input::dsu {

namespace {

constexpr std::uint8_t kClientMagic[4] = {'D', 'S', 'U', 'C'};
constexpr std::uint8_t kServerMagic[4] = {'D', 'S', 'U', 'S'};
constexpr std::size_t kCrcOffset = 8;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadPerDegree = kPi / 180.0f;

constexpr std::array<std::uint32_t, 256> make_crc_table() noexcept {
    std::array<std::uint32_t, 256> table{};
    for (std::uint32_t n = 0; n < 256u; ++n) {
        std::uint32_t c = n;
        for (int k = 0; k < 8; ++k) c = (c & 1u) != 0u ? 0xEDB88320u ^ (c >> 1u) : c >> 1u;
        table[n] = c;
    }
    return table;
}
constexpr std::array<std::uint32_t, 256> kCrcTable = make_crc_table();

void put_u16(std::uint8_t* d, std::uint16_t v) noexcept {
    d[0] = static_cast<std::uint8_t>(v);
    d[1] = static_cast<std::uint8_t>(v >> 8u);
}
void put_u32(std::uint8_t* d, std::uint32_t v) noexcept {
    for (int i = 0; i < 4; ++i) d[i] = static_cast<std::uint8_t>(v >> (8 * i));
}
std::uint16_t get_u16(const std::uint8_t* d) noexcept {
    return static_cast<std::uint16_t>(d[0] | (d[1] << 8u));
}
std::uint32_t get_u32(const std::uint8_t* d) noexcept {
    return static_cast<std::uint32_t>(d[0]) | (static_cast<std::uint32_t>(d[1]) << 8u) |
           (static_cast<std::uint32_t>(d[2]) << 16u) | (static_cast<std::uint32_t>(d[3]) << 24u);
}
std::uint64_t get_u64(const std::uint8_t* d) noexcept {
    return static_cast<std::uint64_t>(get_u32(d)) | (static_cast<std::uint64_t>(get_u32(d + 4)) << 32u);
}
float get_f32(const std::uint8_t* d) noexcept {
    const std::uint32_t bits = get_u32(d);
    float value = 0.0f;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

// The CRC over a packet whose CRC field is still to be filled (or is treated as zero).
std::uint32_t packet_crc(std::span<const std::uint8_t> packet) noexcept {
    std::uint32_t c = 0xFFFFFFFFu;
    for (std::size_t i = 0; i < packet.size(); ++i) {
        const std::uint8_t byte = i >= kCrcOffset && i < kCrcOffset + 4u ? 0u : packet[i];
        c = kCrcTable[(c ^ byte) & 0xFFu] ^ (c >> 8u);
    }
    return c ^ 0xFFFFFFFFu;
}

// Writes the client header and message type over `size` bytes and finishes the CRC.
std::size_t finish_client(std::uint32_t client_id, std::uint32_t type, std::size_t size,
                          std::span<std::uint8_t> out) noexcept {
    std::uint8_t* d = out.data();
    std::memcpy(d, kClientMagic, 4);
    put_u16(d + 4, kProtocolVersion);
    put_u16(d + 6, static_cast<std::uint16_t>(size - kHeaderBytes));
    put_u32(d + 8, 0u);
    put_u32(d + 12, client_id);
    put_u32(d + 16, type);
    put_u32(d + 8, packet_crc(out.first(size)));
    return size;
}

std::string_view trim(std::string_view s) noexcept {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t' || s.front() == '\r')) s.remove_prefix(1);
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t' || s.back() == '\r')) s.remove_suffix(1);
    return s;
}
bool iequals(std::string_view a, std::string_view b) noexcept {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const auto lower = [](char c) { return c >= 'A' && c <= 'Z' ? static_cast<char>(c - 'A' + 'a') : c; };
        if (lower(a[i]) != lower(b[i])) return false;
    }
    return true;
}
bool parse_bool(std::string_view v, bool& out) noexcept {
    if (v == "1" || iequals(v, "true") || iequals(v, "on") || iequals(v, "yes")) {
        out = true;
        return true;
    }
    if (v == "0" || iequals(v, "false") || iequals(v, "off") || iequals(v, "no")) {
        out = false;
        return true;
    }
    return false;
}
template <typename T>
bool parse_int(std::string_view v, T& out) noexcept {
    T value{};
    const auto [end, error] = std::from_chars(v.data(), v.data() + v.size(), value);
    if (error != std::errc{} || end != v.data() + v.size()) return false;
    out = value;
    return true;
}
bool parse_float(std::string_view v, float& out) noexcept {
    // from_chars for float is not in every standard library this builds with; parse a plain decimal by hand.
    if (v.empty()) return false;
    bool negative = false;
    std::size_t i = 0;
    if (v[0] == '-' || v[0] == '+') {
        negative = v[0] == '-';
        ++i;
    }
    double value = 0.0;
    double scale = 0.0;
    bool digits = false;
    for (; i < v.size(); ++i) {
        const char c = v[i];
        if (c >= '0' && c <= '9') {
            digits = true;
            if (scale == 0.0) {
                value = value * 10.0 + (c - '0');
            } else {
                value += (c - '0') * scale;
                scale *= 0.1;
            }
        } else if (c == '.' && scale == 0.0) {
            scale = 0.1;
        } else {
            return false;
        }
    }
    if (!digits) return false;
    out = static_cast<float>(negative ? -value : value);
    return std::isfinite(out);
}

}  // namespace

std::uint32_t crc32(std::span<const std::uint8_t> data) noexcept {
    std::uint32_t c = 0xFFFFFFFFu;
    for (const std::uint8_t byte : data) c = kCrcTable[(c ^ byte) & 0xFFu] ^ (c >> 8u);
    return c ^ 0xFFFFFFFFu;
}

std::size_t build_version_request(std::uint32_t client_id, std::span<std::uint8_t> out) noexcept {
    constexpr std::size_t kSize = kHeaderBytes + 4u;
    if (out.size() < kSize) return 0;
    return finish_client(client_id, kMessageVersion, kSize, out);
}

std::size_t build_ports_request(std::uint32_t client_id, std::span<const std::uint8_t> slots,
                                std::span<std::uint8_t> out) noexcept {
    const std::size_t count = std::min(slots.size(), kSlotCount);
    const std::size_t size = kHeaderBytes + 4u + 4u + count;
    if (out.size() < size) return 0;
    put_u32(out.data() + 20, static_cast<std::uint32_t>(count));
    for (std::size_t i = 0; i < count; ++i) out[24 + i] = slots[i];
    return finish_client(client_id, kMessagePorts, size, out);
}

std::size_t build_pad_data_request(std::uint32_t client_id, std::uint8_t slot, bool all_slots,
                                   std::span<std::uint8_t> out) noexcept {
    constexpr std::size_t kSize = kHeaderBytes + 4u + 8u;
    if (out.size() < kSize) return 0;
    out[20] = all_slots ? 0u : 1u;  // registration: 0 every pad, 1 by slot id, 2 by MAC
    out[21] = all_slots ? 0u : slot;
    std::fill_n(out.data() + 22, 6, std::uint8_t{0});
    return finish_client(client_id, kMessagePadData, kSize, out);
}

std::optional<std::uint32_t> validate_server_packet(std::span<const std::uint8_t> packet) noexcept {
    if (packet.size() < kHeaderBytes + 4u) return std::nullopt;
    if (std::memcmp(packet.data(), kServerMagic, 4) != 0) return std::nullopt;
    // Servers answer with the version they speak; 1001 is the only one in use.
    if (get_u16(packet.data() + 4) != kProtocolVersion) return std::nullopt;
    const std::size_t length = get_u16(packet.data() + 6);
    if (length < 4u || kHeaderBytes + length > packet.size()) return std::nullopt;
    const std::span<const std::uint8_t> message = packet.first(kHeaderBytes + length);
    if (packet_crc(message) != get_u32(packet.data() + kCrcOffset)) return std::nullopt;
    return get_u32(packet.data() + 16);
}

std::optional<PadData> parse_pad_data(std::span<const std::uint8_t> packet) noexcept {
    const std::optional<std::uint32_t> type = validate_server_packet(packet);
    if (!type || *type != kMessagePadData) return std::nullopt;
    if (kHeaderBytes + get_u16(packet.data() + 6) < kPadDataBytes) return std::nullopt;
    const std::uint8_t* d = packet.data();
    PadData pad{};
    pad.slot = d[20];
    pad.slot_state = d[21];
    pad.model = d[22];
    pad.active = d[31] != 0u;
    pad.packet_number = get_u32(d + 32);
    pad.motion_timestamp_us = get_u64(d + 68);
    for (std::size_t i = 0; i < 3; ++i) {
        pad.accel_g[i] = get_f32(d + 76 + 4 * i);
        pad.gyro_dps[i] = get_f32(d + 88 + 4 * i);
    }
    for (const float v : pad.accel_g) {
        if (!std::isfinite(v)) return std::nullopt;
    }
    for (const float v : pad.gyro_dps) {
        if (!std::isfinite(v)) return std::nullopt;
    }
    return pad;
}

switch_hid::ImuSample to_imu_sample(const PadData& pad, const DsuAxisFlips& flips) noexcept {
    switch_hid::ImuSample sample{};
    const float accel_sign = flips.accel ? 1.0f : -1.0f;
    for (std::size_t i = 0; i < 3; ++i) sample.accel[i] = accel_sign * pad.accel_g[i];
    sample.gyro[0] = (flips.pitch ? -1.0f : 1.0f) * pad.gyro_dps[0] * kRadPerDegree;
    sample.gyro[1] = (flips.yaw ? 1.0f : -1.0f) * pad.gyro_dps[1] * kRadPerDegree;
    sample.gyro[2] = (flips.roll ? 1.0f : -1.0f) * pad.gyro_dps[2] * kRadPerDegree;
    return sample;
}

float sample_dt(std::uint64_t previous_us, std::uint64_t now_us, float host_dt) noexcept {
    if (previous_us != 0u && now_us != previous_us) {
        std::uint64_t step = now_us - previous_us;
        // A server that sends the controller's own 32-bit microsecond clock (SC2DSU) wraps it every 71.6 minutes.
        if (now_us < previous_us && previous_us <= 0xFFFFFFFFull && now_us <= 0xFFFFFFFFull) {
            step = (now_us + 0x1'0000'0000ull) - previous_us;
        }
        if (step >= 500u && step <= 50'000u) return static_cast<float>(step) * 1e-6f;
    }
    return std::clamp(host_dt, 0.0f, 0.05f);
}

DsuConfig parse_config(std::string_view text) noexcept {
    DsuConfig config{};
    while (!text.empty()) {
        const std::size_t end = text.find('\n');
        std::string_view line = text.substr(0, end);
        text = end == std::string_view::npos ? std::string_view{} : text.substr(end + 1);
        if (const std::size_t comment = line.find_first_of("#;"); comment != std::string_view::npos) {
            line = line.substr(0, comment);
        }
        const std::size_t eq = line.find('=');
        if (eq == std::string_view::npos) continue;
        const std::string_view key = trim(line.substr(0, eq));
        const std::string_view value = trim(line.substr(eq + 1));
        bool flag = false;
        int number = 0;
        float real = 0.0f;
        if (iequals(key, "enabled") && parse_bool(value, flag)) {
            config.enabled = flag;
        } else if (iequals(key, "server") && !value.empty()) {
            config.server = std::string(value);
        } else if (iequals(key, "port") && parse_int(value, number) && number > 0 && number <= 65535) {
            config.port = static_cast<std::uint16_t>(number);
        } else if (iequals(key, "slot")) {
            if (iequals(value, "auto")) {
                config.slot = -1;
            } else if (parse_int(value, number) && number >= -1 && number < static_cast<int>(kSlotCount)) {
                config.slot = number;
            }
        } else if (iequals(key, "gyro_pointer") && parse_bool(value, flag)) {
            config.gyro_pointer = flag;
        } else if (iequals(key, "gyro_sensitivity") && parse_float(value, real)) {
            config.gyro_sensitivity = std::clamp(real, 0.2f, 5.0f);
        } else if (iequals(key, "shake")) {
            if (iequals(value, "off") || value == "0") config.shake = ShakeTarget::Off;
            if (iequals(value, "remote")) config.shake = ShakeTarget::Remote;
            if (iequals(value, "nunchuk")) config.shake = ShakeTarget::Nunchuk;
        } else if (iequals(key, "shake_sensitivity") && parse_int(value, number)) {
            config.shake_sensitivity = static_cast<std::uint8_t>(std::clamp(number, 1, 5));
        } else if (iequals(key, "tilt") && parse_bool(value, flag)) {
            config.tilt = flag;
        } else if (iequals(key, "invert_pitch") && parse_bool(value, flag)) {
            config.flips.pitch = flag;
        } else if (iequals(key, "invert_yaw") && parse_bool(value, flag)) {
            config.flips.yaw = flag;
        } else if (iequals(key, "invert_roll") && parse_bool(value, flag)) {
            config.flips.roll = flag;
        } else if (iequals(key, "invert_accel") && parse_bool(value, flag)) {
            config.flips.accel = flag;
        }
    }
    return config;
}

std::string default_config_text() {
    return "# Nebula DSU (cemuhook) motion client.\n"
           "# Reads gyro and accelerometer from a DSU server such as SC2DSU (Steam Controllers), BetterJoy or\n"
           "# DS4Windows and gives them to the controller you play with when it has no motion sensors of its own\n"
           "# (for example Steam Input's virtual Xbox controller). Buttons and sticks still come from that controller.\n"
           "# Changes apply while the game runs.\n"
           "\n"
           "enabled=1\n"
           "server=127.0.0.1\n"
           "port=26760\n"
           "# The server's controller slot (0-3), or auto: the first slot that sends motion.\n"
           "slot=auto\n"
           "\n"
           "# Point with the gyro; 1.0 = one picture width per 40 degrees of turn (0.2 .. 5).\n"
           "gyro_pointer=1\n"
           "gyro_sensitivity=1.0\n"
           "# A shake of the controller as a shake of the Wii Remote (spin) or Nunchuk, or off.\n"
           "shake=remote\n"
           "# 1 needs a strong shake .. 5 a light one.\n"
           "shake_sensitivity=3\n"
           "# The tilt sections (Star Ball, Ray surfing) steer by the controller's tilt.\n"
           "tilt=1\n"
           "\n"
           "# Turn an axis around if the server sends it the other way.\n"
           "invert_pitch=0\n"
           "invert_yaw=0\n"
           "invert_roll=0\n"
           "invert_accel=0\n";
}

void apply_server_override(std::string_view value, DsuConfig& config) noexcept {
    value = trim(value);
    if (value.empty()) return;
    const std::size_t colon = value.rfind(':');
    std::string_view host = value;
    if (colon != std::string_view::npos) {
        int port = 0;
        if (parse_int(value.substr(colon + 1), port) && port > 0 && port <= 65535) {
            config.port = static_cast<std::uint16_t>(port);
        }
        host = value.substr(0, colon);
    }
    if (!host.empty()) config.server = std::string(host);
}

}  // namespace galaxy::input::dsu
