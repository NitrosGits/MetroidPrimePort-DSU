#pragma once

// DSU ("cemuhook") controller client: pads that another program serves over UDP
// (NSO GC Driver, DS4Windows, BetterJoy, phone and Steam Deck apps; the protocol
// Dolphin's DSU client reads). Every connected DSU slot becomes an SDL virtual
// gamepad named "DSU slot <n>", so it takes a port, gets the pad type's default
// mapping and is remapped like any other controller (F1 > Controls > Controller
// settings > DSU controllers).
//
// The packet code (Proto) is pure so tests/port_dsu.cpp links it alone. A network
// thread keeps the newest pad data per slot; Update(), on the game thread before
// each input poll, attaches, updates and detaches the virtual pads.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace PortDsu {

constexpr int kSlotCount = 4;
constexpr const char* kDefaultServer = "127.0.0.1:26760";

// What the server's controllers are, which decides how the DS4-shaped DSU buttons
// become the virtual pad's (PortInput::Settings::dsuLayout keeps the number).
enum Layout : int {
  // A GameCube pad behind the server (NSO GC Driver: A/B/X/Y = cross/circle/square/
  // triangle, Z = R3, L/R = analog L2/R2 with L1/R1 at the click). The virtual pad
  // reports as Nintendo's GameCube adapter, so it gets the GameCube mapping.
  kLayoutGameCube = 0,
  // A DS4-shaped pad (DS4Windows, BetterJoy, phone apps): buttons by position.
  kLayoutPlayStation = 1,
  kLayoutCount
};

namespace Proto {
constexpr uint16_t kVersion = 1001;
constexpr uint32_t kTypePorts = 0x100001;
constexpr uint32_t kTypePadData = 0x100002;
constexpr std::size_t kHeaderSize = 16;
constexpr std::size_t kPadDataSize = kHeaderSize + 84;
constexpr std::size_t kPortInfoSize = kHeaderSize + 16;

uint32_t Crc32(const uint8_t* data, std::size_t size);

// A client message: header, type and payload, signed.
std::vector< uint8_t > BuildRequest(uint32_t clientId, uint32_t type, const std::vector< uint8_t >& payload);

// One slot's pad data. Sticks are 0-255 with 128 centred and larger = right/up;
// triggers 0-255. buttons1/buttons2 are the DS4 bits as the protocol sends them.
struct Pad {
  int slot = 0;
  bool connected = false;
  uint8_t model = 0;
  uint8_t connection = 0;
  uint8_t battery = 0;
  uint8_t buttons1 = 0;
  uint8_t buttons2 = 0;
  uint8_t home = 0;
  uint8_t touch = 0;
  uint8_t lx = 128, ly = 128, rx = 128, ry = 128;
  uint8_t l2 = 0, r2 = 0;
};

// A server's pad data message: false for anything else, a bad CRC or a short packet.
bool ParsePadData(const uint8_t* data, std::size_t size, Pad& out);
// A server's slot info message (the reply to kTypePorts): which slot, and whether a
// pad is connected there. Servers that only send pad data when it changes (NSO GC
// Driver) answer this every second, so it keeps an idle pad alive.
bool ParsePortInfo(const uint8_t* data, std::size_t size, int& slot, bool& connected);
} // namespace Proto

// "host[:port]" (port 26760 when left out; [v6] in brackets) split in two; false if empty.
bool SplitServer(const std::string& server, std::string& host, std::string& port);

// Game thread, before each input poll: starts or stops the client from the
// settings and keeps the virtual pads in step with what the server sends.
void Update();

// For F1: the server line, and one line per connected slot.
std::string ServerStatus();
std::vector< std::string > SlotStatus();

} // namespace PortDsu
