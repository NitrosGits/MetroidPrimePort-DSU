// The DSU protocol's packets (version 1001, little endian): a 16-byte header
// ("DSUC" from the client / "DSUS" from the server, u16 version, u16 length of
// what follows the header, u32 CRC32 of the whole message with this field zero,
// u32 sender id), then a u32 message type and its payload.

#include "port_dsu.h"

#include <array>
#include <cstring>

namespace PortDsu {
namespace Proto {
namespace {

void PutU16(uint8_t* p, uint16_t v) {
  p[0] = uint8_t(v);
  p[1] = uint8_t(v >> 8);
}

void PutU32(uint8_t* p, uint32_t v) {
  PutU16(p, uint16_t(v));
  PutU16(p + 2, uint16_t(v >> 16));
}

uint16_t GetU16(const uint8_t* p) { return uint16_t(p[0] | p[1] << 8); }

uint32_t GetU32(const uint8_t* p) { return uint32_t(GetU16(p)) | uint32_t(GetU16(p + 2)) << 16; }

} // namespace

uint32_t Crc32(const uint8_t* data, std::size_t size) {
  static const std::array< uint32_t, 256 > table = [] {
    std::array< uint32_t, 256 > t{};
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t c = i;
      for (int k = 0; k < 8; ++k) {
        c = (c & 1) != 0 ? 0xEDB88320u ^ (c >> 1) : c >> 1;
      }
      t[i] = c;
    }
    return t;
  }();
  uint32_t crc = 0xFFFFFFFFu;
  for (std::size_t i = 0; i < size; ++i) {
    crc = table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  }
  return crc ^ 0xFFFFFFFFu;
}

std::vector< uint8_t > BuildRequest(uint32_t clientId, uint32_t type, const std::vector< uint8_t >& payload) {
  std::vector< uint8_t > out(kHeaderSize + 4 + payload.size());
  std::memcpy(out.data(), "DSUC", 4);
  PutU16(&out[4], kVersion);
  PutU16(&out[6], uint16_t(out.size() - kHeaderSize));
  PutU32(&out[12], clientId);
  PutU32(&out[16], type);
  std::memcpy(&out[20], payload.data(), payload.size());
  PutU32(&out[8], Crc32(out.data(), out.size()));
  return out;
}

namespace {
// A server message of this type, at least minSize long, whose CRC (over the whole
// message, a newer server's extra fields too) checks out.
bool Valid(const uint8_t* data, std::size_t size, uint32_t type, std::size_t minSize) {
  if (size < minSize || std::memcmp(data, "DSUS", 4) != 0 || GetU16(data + 4) > kVersion ||
      GetU32(data + 16) != type) {
    return false;
  }
  const std::size_t length = kHeaderSize + GetU16(data + 6);
  uint8_t packet[1024];
  if (length > size || length < minSize || length > sizeof(packet)) {
    return false;
  }
  std::memcpy(packet, data, length);
  std::memset(packet + 8, 0, 4);
  return Crc32(packet, length) == GetU32(data + 8);
}
} // namespace

bool ParsePortInfo(const uint8_t* data, std::size_t size, int& slot, bool& connected) {
  if (!Valid(data, size, kTypePorts, kPortInfoSize)) {
    return false;
  }
  slot = data[20];
  connected = data[21] == 2;
  return true;
}

bool ParsePadData(const uint8_t* data, std::size_t size, Pad& out) {
  if (!Valid(data, size, kTypePadData, kPadDataSize)) {
    return false;
  }
  out.slot = data[20];
  // pad_state 2 is connected, and byte 31 says the pad is active.
  out.connected = data[21] == 2 && data[31] != 0;
  out.model = data[22];
  out.connection = data[23];
  out.battery = data[30];
  out.buttons1 = data[36];
  out.buttons2 = data[37];
  out.home = data[38];
  out.touch = data[39];
  out.lx = data[40];
  out.ly = data[41];
  out.rx = data[42];
  out.ry = data[43];
  // The analog values run d-pad, face buttons, R1, L1, then R2 and L2.
  out.r2 = data[54];
  out.l2 = data[55];
  return true;
}

} // namespace Proto

bool SplitServer(const std::string& server, std::string& host, std::string& port) {
  const std::size_t b = server.find_first_not_of(" \t");
  if (b == std::string::npos) {
    return false;
  }
  const std::string s = server.substr(b, server.find_last_not_of(" \t") - b + 1);
  const std::size_t close = s.find(']');
  const std::size_t colon = s.rfind(':');
  // A bare IPv6 address has more than one colon and no port.
  const bool hasPort = colon != std::string::npos && (s[0] == '[' ? colon > close : s.find(':') == colon);
  host = hasPort ? s.substr(0, colon) : s;
  port = hasPort ? s.substr(colon + 1) : "26760";
  if (host.size() > 2 && host.front() == '[' && host.back() == ']') {
    host = host.substr(1, host.size() - 2);
  }
  return !host.empty() && !port.empty();
}

} // namespace PortDsu
