#pragma once

// Motion from a cemuhook DSU server (dsu_protocol.h), such as SC2DSU for the Steam Controllers. One worker thread
// subscribes to the server over UDP, renews the subscription every second, and runs the chosen slot's samples through
// the same motion filter the HID controllers use (switch_controller.h SwitchMotion: gyro bias at rest, gravity, shake,
// world-space pointer). The input consumer copies the result and attaches it to the controller in use when that
// controller has no motion sensors of its own (native_host.cpp), so Steam Input's virtual Xbox controller points,
// shakes and tilts with the Steam Controller's gyro while its buttons and sticks still arrive through XInput.
//
// Settings: dsu.ini in Nebula's data folder (%LOCALAPPDATA%\Nebula), written with the defaults when missing and read
// again when it changes. NEBULA_DSU=0 turns the client off; NEBULA_DSU_SERVER=host[:port] overrides the server.

#include "galaxy/dsu_protocol.h"

#include <array>
#include <cstdint>

namespace galaxy::host {

struct DsuMotionState {
    // Motion arrived within the last half second.
    bool live = false;
    std::uint8_t server_slot = 0;
    // Pointer movement since the previous consuming read (pointer units, 2 = picture width; up positive).
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    // A shake held until this steady-clock microsecond (controller_hid_now_us), 0: none.
    std::uint64_t shake_until_us = 0;
    // Controller up (gravity reaction, SDL frame), valid once the filter has settled.
    std::array<float, 3> gravity{0.0f, 1.0f, 0.0f};
    bool gravity_valid = false;
    // The settings that decide how the consumer uses it.
    bool gyro_pointer = true;
    bool tilt = true;
    input::dsu::ShakeTarget shake = input::dsu::ShakeTarget::Remote;
};

// Starts the worker on first use (when enabled); later calls are free.
void ensure_dsu_client_started() noexcept;
// Copies the latest motion; `consume` clears the pointer accumulator. False while the client is off or has not
// started.
bool read_dsu_motion(DsuMotionState& out, bool consume) noexcept;

}  // namespace galaxy::host
