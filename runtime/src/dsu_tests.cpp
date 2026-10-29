// Cemuhook DSU client (dsu_protocol.h): the CRC, the client's requests, pad data as SC2DSU writes it, the motion
// conversion (checked against the 2026 Steam Controller's own HID decoder: the same raw sample read natively and
// through SC2DSU must agree), the motion clock and the settings file.

#include "galaxy/dsu_protocol.h"
#include "galaxy/steam_controller.h"
#include "galaxy/switch_controller.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

namespace {

namespace dsu = galaxy::input::dsu;
namespace sv = galaxy::input::steam;
namespace sw = galaxy::input::switch_hid;
using Bytes = std::vector<std::uint8_t>;

int g_failures = 0;

void expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAILED: " << message << '\n';
        ++g_failures;
    }
}

bool close_to(float a, float b, float tolerance = 1e-4f) {
    return std::fabs(a - b) <= tolerance;
}

void put16(Bytes& b, std::size_t at, std::int32_t v) {
    b[at] = static_cast<std::uint8_t>(v);
    b[at + 1] = static_cast<std::uint8_t>(v >> 8);
}
void put32(Bytes& b, std::size_t at, std::uint32_t v) {
    for (std::size_t i = 0; i < 4; ++i) b[at + i] = static_cast<std::uint8_t>(v >> (8 * i));
}
void put64(Bytes& b, std::size_t at, std::uint64_t v) {
    for (std::size_t i = 0; i < 8; ++i) b[at + i] = static_cast<std::uint8_t>(v >> (8 * i));
}
void putf(Bytes& b, std::size_t at, float v) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    put32(b, at, bits);
}
std::uint32_t get32(const Bytes& b, std::size_t at) {
    return static_cast<std::uint32_t>(b[at]) | (static_cast<std::uint32_t>(b[at + 1]) << 8) |
           (static_cast<std::uint32_t>(b[at + 2]) << 16) | (static_cast<std::uint32_t>(b[at + 3]) << 24);
}
std::uint32_t crc_with_field_zeroed(Bytes b) {
    put32(b, 8, 0);
    return dsu::crc32(b);
}

struct Motion {
    std::array<float, 3> accel_g{};
    std::array<float, 3> gyro_dps{};
};

// A pad data packet laid out as SC2DSU's dsu.rs writes it: the 20-byte header with the message type, the 11-byte slot
// header, then the body (connected flag, packet number, buttons, sticks, analog buttons, touches, the motion clock
// and the six motion floats).
Bytes server_pad_data(std::uint8_t slot, std::uint32_t packet, std::uint64_t timestamp_us, const Motion& m) {
    Bytes b(100, 0);
    std::memcpy(b.data(), "DSUS", 4);
    put16(b, 4, 1001);
    put16(b, 6, static_cast<std::int32_t>(b.size() - 16));
    put32(b, 12, 0x12345678u);  // server id
    put32(b, 16, dsu::kMessagePadData);
    b[20] = slot;
    b[21] = 2;  // connected
    b[22] = 2;  // full gyro
    b[23] = 1;  // USB
    for (std::size_t i = 0; i < 6; ++i) b[24 + i] = static_cast<std::uint8_t>(0xA0 + i);
    b[30] = 0;     // battery n/a
    b[31] = 1;     // active
    put32(b, 32, packet);
    b[40] = b[41] = b[42] = b[43] = 128;  // sticks centred
    put64(b, 68, timestamp_us);
    for (std::size_t i = 0; i < 3; ++i) {
        putf(b, 76 + 4 * i, m.accel_g[i]);
        putf(b, 88 + 4 * i, m.gyro_dps[i]);
    }
    put32(b, 8, crc_with_field_zeroed(b));
    return b;
}

void test_crc() {
    const char* check = "123456789";
    const std::span<const std::uint8_t> bytes(reinterpret_cast<const std::uint8_t*>(check), 9);
    expect(dsu::crc32(bytes) == 0xCBF43926u, "CRC-32 check value");
    expect(dsu::crc32({}) == 0u, "CRC-32 of nothing");
}

void test_requests() {
    std::array<std::uint8_t, 64> out{};
    const std::size_t version = dsu::build_version_request(0xCAFEu, out);
    Bytes v(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(version));
    expect(version == 20u && std::memcmp(v.data(), "DSUC", 4) == 0, "version request: size and magic");
    expect(v[4] == 0xE9 && v[5] == 0x03 && v[6] == 4 && v[7] == 0, "version request: protocol 1001, length 4");
    expect(get32(v, 12) == 0xCAFEu && get32(v, 16) == dsu::kMessageVersion, "version request: id and type");
    expect(get32(v, 8) == crc_with_field_zeroed(v), "version request: CRC");

    constexpr std::uint8_t kSlots[] = {0, 1, 2, 3, 4};
    const std::size_t ports = dsu::build_ports_request(7u, kSlots, out);
    Bytes p(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(ports));
    expect(ports == 28u && get32(p, 20) == 4u && p[24] == 0 && p[27] == 3, "ports request: at most four slots");
    expect(get32(p, 8) == crc_with_field_zeroed(p), "ports request: CRC");

    const std::size_t by_slot = dsu::build_pad_data_request(7u, 2, false, out);
    Bytes s(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(by_slot));
    expect(by_slot == 28u && get32(s, 16) == dsu::kMessagePadData && s[20] == 1 && s[21] == 2,
           "pad data request by slot");
    expect(s[6] == 12 && get32(s, 8) == crc_with_field_zeroed(s), "pad data request: length and CRC");
    const std::size_t all = dsu::build_pad_data_request(7u, 3, true, out);
    expect(all == 28u && out[20] == 0 && out[21] == 0, "pad data request for every slot");

    std::array<std::uint8_t, 8> small{};
    expect(dsu::build_version_request(1u, small) == 0u && dsu::build_pad_data_request(1u, 0, true, small) == 0u,
           "a buffer too small writes nothing");
}

void test_pad_data() {
    const Motion m{{0.25f, -1.0f, 0.5f}, {10.0f, -20.0f, 30.0f}};
    const Bytes packet = server_pad_data(1, 42, 123'456'789ull, m);
    expect(dsu::validate_server_packet(packet) == dsu::kMessagePadData, "a server packet validates");
    const auto pad = dsu::parse_pad_data(packet);
    expect(pad.has_value(), "pad data parses");
    if (pad) {
        expect(pad->slot == 1 && pad->slot_state == 2 && pad->model == 2 && pad->active, "slot header");
        expect(pad->packet_number == 42 && pad->motion_timestamp_us == 123'456'789ull, "packet number and clock");
        expect(close_to(pad->accel_g[0], 0.25f) && close_to(pad->accel_g[1], -1.0f) &&
                   close_to(pad->accel_g[2], 0.5f),
               "accelerometer as sent");
        expect(close_to(pad->gyro_dps[0], 10.0f) && close_to(pad->gyro_dps[1], -20.0f) &&
                   close_to(pad->gyro_dps[2], 30.0f),
               "gyroscope as sent");
    }

    Bytes bad_crc = packet;
    bad_crc[80] ^= 0x01u;
    expect(!dsu::parse_pad_data(bad_crc), "a changed byte fails the CRC");
    Bytes client = packet;
    client[3] = 'C';
    put32(client, 8, crc_with_field_zeroed(client));
    expect(!dsu::validate_server_packet(client), "a client packet is not the server's");
    Bytes version = packet;
    put16(version, 4, 1000);
    put32(version, 8, crc_with_field_zeroed(version));
    expect(!dsu::validate_server_packet(version), "another protocol version");
    expect(!dsu::parse_pad_data(Bytes(packet.begin(), packet.begin() + 60)), "a short packet");
    Bytes overlong = packet;
    put16(overlong, 6, 200);
    expect(!dsu::validate_server_packet(overlong), "a length past the packet");
    Bytes other = packet;
    put32(other, 16, dsu::kMessagePorts);
    put32(other, 8, crc_with_field_zeroed(other));
    expect(dsu::validate_server_packet(other) == dsu::kMessagePorts && !dsu::parse_pad_data(other),
           "port info is not pad data");
    const Bytes nan_packet = server_pad_data(0, 1, 1, Motion{{0.0f, std::nanf(""), 0.0f}, {}});
    expect(!dsu::parse_pad_data(nan_packet), "a NaN sample is refused");
    // Trailing bytes past the stated length are ignored (and not part of the CRC).
    Bytes padded = packet;
    padded.push_back(0xEE);
    expect(dsu::parse_pad_data(padded).has_value(), "bytes after the message are ignored");
}

// The 2026 Steam Controller's raw IMU (its own frame: x right, y away from the player, z up; 16384 counts = 1 g or
// 1000 deg/s) as SC2DSU's default axis map sends it (config.rs: gyro (x, -z, y), accel (-x, -z, y)).
Motion sc2dsu_motion(const std::array<int, 6>& raw) {
    const auto g = [](int v) { return static_cast<float>(v) / 32768.0f * 2.0f; };
    const auto dps = [](int v) { return static_cast<float>(v) / 32768.0f * 2000.0f; };
    return Motion{{-g(raw[0]), -g(raw[2]), g(raw[1])}, {dps(raw[3]), -dps(raw[5]), dps(raw[4])}};
}

// The same raw sample in Nebula's own 2026 Steam Controller report (steam_controller.h), decoded natively.
sw::ImuSample native_sample(const std::array<int, 6>& raw) {
    Bytes r(46, 0);
    r[0] = sv::kReportStateBle;
    put32(r, 30, 1000);
    for (std::size_t i = 0; i < 6; ++i) put16(r, 34 + 2 * i, raw[i]);
    sv::SensorClock clock;
    sw::DecodedState state;
    if (!sv::decode_report(r, clock, state) || state.imu_count != 1) return {};
    return state.imu[0];
}

bool same_sample(const sw::ImuSample& a, const sw::ImuSample& b) {
    for (std::size_t i = 0; i < 3; ++i) {
        if (!close_to(a.accel[i], b.accel[i], 1e-3f) || !close_to(a.gyro[i], b.gyro[i], 1e-3f)) return false;
    }
    return true;
}

void test_steam_controller_through_sc2dsu() {
    const std::array<std::array<int, 6>, 6> samples{{
        {0, 0, 16384, 0, 0, 0},                 // flat on a table
        {16384, 0, 0, 0, 0, 0},                 // on its right side
        {0, -16384, 0, 0, 0, 0},                // standing on its grips
        {0, 0, 16384, 1638, 0, 0},              // pitching
        {0, 0, 16384, 0, 0, -2048},             // yawing
        {1200, -3400, 15800, -900, 2400, 3100}  // anything
    }};
    for (const auto& raw : samples) {
        const sw::ImuSample native = native_sample(raw);
        const Bytes packet = server_pad_data(0, 1, 1000, sc2dsu_motion(raw));
        const auto pad = dsu::parse_pad_data(packet);
        expect(pad.has_value() && same_sample(dsu::to_imu_sample(*pad), native),
               "SC2DSU's Steam Controller motion matches Nebula's native reading");
    }

    // At rest, flat: DSU sends -1 g on Y (as Dolphin and Cemu read it); Nebula's frame is +1 g up.
    const auto flat = dsu::parse_pad_data(server_pad_data(0, 1, 1000, sc2dsu_motion({0, 0, 16384, 0, 0, 0})));
    expect(flat && close_to(flat->accel_g[1], -1.0f, 1e-3f), "DSU: -1 g on Y lying flat");
    expect(flat && close_to(dsu::to_imu_sample(*flat).accel[1], 1.0f, 1e-3f), "Nebula: +1 g up lying flat");

    // A turn to the right is positive DSU yaw and negative (clockwise) about Nebula's up axis.
    dsu::PadData right{};
    right.gyro_dps = {0.0f, 90.0f, 0.0f};
    expect(dsu::to_imu_sample(right).gyro[1] < 0.0f, "yaw right: clockwise about up");
    dsu::PadData nose_up{};
    nose_up.gyro_dps = {90.0f, 0.0f, 0.0f};
    expect(dsu::to_imu_sample(nose_up).gyro[0] > 0.0f, "pitch up: counter-clockwise about right");
    expect(close_to(dsu::to_imu_sample(nose_up).gyro[0], 3.14159265f / 2.0f, 1e-4f), "degrees become radians");

    dsu::DsuAxisFlips flips{};
    flips.yaw = true;
    flips.accel = true;
    dsu::PadData any{};
    any.accel_g = {0.0f, -1.0f, 0.0f};
    any.gyro_dps = {0.0f, 90.0f, 0.0f};
    const sw::ImuSample flipped = dsu::to_imu_sample(any, flips);
    expect(flipped.gyro[1] > 0.0f && close_to(flipped.accel[1], -1.0f), "inverted yaw and accelerometer");
}

// End to end: the shared motion filter turns SC2DSU's samples into pointer movement the right way round.
void test_pointer_direction() {
    sw::SwitchMotion motion;
    sw::MotionSettings settings{};
    settings.shake = false;
    const float dt = 0.004f;
    std::uint64_t t = 1'000'000;
    const auto feed = [&](const Motion& m, float seconds, float& x, float& y) {
        for (int i = 0; i < static_cast<int>(seconds / dt); ++i) {
            const auto pad = dsu::parse_pad_data(server_pad_data(0, static_cast<std::uint32_t>(i + 1), t, m));
            t += 4000;
            if (!pad) continue;
            const sw::SwitchMotion::Output out = motion.feed(dsu::to_imu_sample(*pad), dt, t, settings);
            x += out.pointer_x;
            y += out.pointer_y;
        }
    };
    float x = 0.0f;
    float y = 0.0f;
    const Motion rest{{0.0f, -1.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    feed(rest, 1.5f, x, y);
    expect(motion.gravity_ready() && close_to(motion.gravity()[1], 1.0f, 0.05f), "gravity settles up at rest");
    x = y = 0.0f;
    feed(Motion{{0.0f, -1.0f, 0.0f}, {0.0f, 40.0f, 0.0f}}, 1.0f, x, y);  // 40 degrees to the right
    expect(x > 1.6f && x < 2.4f && std::fabs(y) < 0.1f, "turning right 40 degrees moves a picture width right");
    x = y = 0.0f;
    feed(Motion{{0.0f, -1.0f, 0.0f}, {20.0f, 0.0f, 0.0f}}, 1.0f, x, y);  // 20 degrees nose up
    expect(y > 0.5f && std::fabs(x) < 0.1f, "pitching up moves the pointer up");
}

void test_sample_dt() {
    expect(close_to(dsu::sample_dt(1000, 5000, 0.02f), 0.004f, 1e-7f), "dt from the server's clock");
    expect(close_to(dsu::sample_dt(0xFFFFF000ull, 0x00000F90ull, 0.02f), 0.00808f, 1e-7f),
           "the 32-bit clock wraps");
    expect(close_to(dsu::sample_dt(0, 5000, 0.02f), 0.02f), "no previous sample: the host's dt");
    expect(close_to(dsu::sample_dt(1000, 1100, 0.008f), 0.008f), "a step too small: the host's dt");
    expect(close_to(dsu::sample_dt(1000, 900'000, 0.3f), 0.05f), "a gap: the host's dt clamped");
    expect(close_to(dsu::sample_dt(0x2'0000'0000ull, 0x1'0000'0000ull, 0.01f), 0.01f),
           "a 64-bit clock going back is no wrap");
}

void test_config() {
    const dsu::DsuConfig defaults{};
    expect(dsu::parse_config("") == defaults, "empty file: the defaults");
    expect(dsu::parse_config(dsu::default_config_text()) == defaults, "the written file reads as the defaults");
    expect(defaults.enabled && defaults.server == "127.0.0.1" && defaults.port == 26760 && defaults.slot == -1,
           "default server and slot");

    const dsu::DsuConfig c = dsu::parse_config(
        "# comment\n"
        "enabled = off\r\n"
        "server=192.168.1.20 ; a remote server\n"
        "port=26761\n"
        "slot=2\n"
        "gyro_pointer=no\n"
        "gyro_sensitivity=1.75\n"
        "shake=Nunchuk\n"
        "shake_sensitivity=9\n"
        "tilt=0\n"
        "invert_pitch=1\n"
        "invert_yaw=true\n"
        "invert_roll=yes\n"
        "invert_accel=on\n"
        "unknown=1\n");
    expect(!c.enabled && c.server == "192.168.1.20" && c.port == 26761 && c.slot == 2, "server settings");
    expect(!c.gyro_pointer && close_to(c.gyro_sensitivity, 1.75f) && c.shake == dsu::ShakeTarget::Nunchuk &&
               c.shake_sensitivity == 5 && !c.tilt,
           "motion settings (shake sensitivity clamped)");
    expect(c.flips.pitch && c.flips.yaw && c.flips.roll && c.flips.accel, "axis flips");

    const dsu::DsuConfig bad = dsu::parse_config("port=0\nport=70000\nslot=7\ngyro_sensitivity=fast\nenabled=maybe\n"
                                                 "gyro_sensitivity=99\nshake=off\n");
    expect(bad.port == 26760 && bad.slot == -1 && bad.enabled, "bad values keep the defaults");
    expect(close_to(bad.gyro_sensitivity, 5.0f) && bad.shake == dsu::ShakeTarget::Off, "sensitivity clamped, shake off");
    expect(dsu::parse_config("slot=1\nslot=auto\n").slot == -1, "slot auto");

    dsu::DsuConfig o{};
    dsu::apply_server_override("10.0.0.5:26800", o);
    expect(o.server == "10.0.0.5" && o.port == 26800, "override host and port");
    dsu::apply_server_override(":27000", o);
    expect(o.server == "10.0.0.5" && o.port == 27000, "override the port only");
    dsu::apply_server_override(" localhost ", o);
    expect(o.server == "localhost" && o.port == 27000, "override the host only");
    dsu::apply_server_override("", o);
    expect(o.server == "localhost", "an empty override changes nothing");
}

}  // namespace

int main() {
    test_crc();
    test_requests();
    test_pad_data();
    test_steam_controller_through_sc2dsu();
    test_pointer_direction();
    test_sample_dt();
    test_config();
    if (g_failures != 0) {
        std::cerr << g_failures << " check(s) failed\n";
        return 1;
    }
    std::cout << "dsu tests passed\n";
    return 0;
}
