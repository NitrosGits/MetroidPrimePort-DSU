#pragma once

// A DSU (cemuhook UDP, protocol 1001) client: motion from a DSU server (a phone
// app, BetterJoy, DS4Windows, SteamDeckGyroDSU...) for gyro aim, so a pad
// without a gyro such as a GameCube controller can still aim by motion. Only
// the motion is read; buttons and sticks stay with the pad.
//
// The packet code (Proto) is pure so tests/port_dsu.cpp links it alone; the
// client runs a background thread that registers for one slot once a second
// and keeps the newest motion.

#include <cstddef>
#include <cstdint>
#include <string>

namespace PortDsu {

constexpr int kDefaultPort = 26760;
constexpr int kSlotCount = 4;
constexpr const char* kDefaultHost = "127.0.0.1";

namespace Proto {
constexpr uint16_t kVersion = 1001;
constexpr uint32_t kTypePadData = 0x100002;
constexpr std::size_t kHeaderSize = 16;
constexpr std::size_t kPadDataRequestSize = kHeaderSize + 12;
constexpr std::size_t kPadDataResponseSize = kHeaderSize + 84;

uint32_t Crc32(const uint8_t* data, std::size_t size);

// A pad data request registering one slot (0-3).
void BuildPadDataRequest(uint8_t (&out)[kPadDataRequestSize], uint32_t clientId, int slot);

// One sample in SDL's sensor axes (x right, y up, z toward the player): gyro in
// rad/s, accel in m/s^2, as SDL_GetGamepadSensorData reports them.
struct Motion {
  int slot = 0;
  bool connected = false;
  uint64_t timestampUs = 0;
  float gyro[3] = {};
  float accel[3] = {};
};

// A server's pad data response: false for anything else, a bad CRC or a short packet.
bool ParsePadData(const uint8_t* data, std::size_t size, Motion& out);
} // namespace Proto

// Starts the client for host:port and slot, or retargets it when they changed;
// cheap to call every tick.
void Use(const std::string& host, int port, int slot);
// Stops the thread and closes the socket (cheap when not running).
void Stop();
// The average gyro and the newest accel since the last call, in SDL's axes.
// False while nothing has arrived for a second (no server, wrong slot).
bool Read(float gyro[3], float accel[3]);
// What the client is doing, for F1.
std::string Status();

} // namespace PortDsu
