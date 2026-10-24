#include "port_dsu.h"
#include "port_test.h"

#include <cstring>

namespace {

using namespace PortDsu;

void PutU32(uint8_t* p, uint32_t v) {
  for (int i = 0; i < 4; ++i) {
    p[i] = uint8_t(v >> (8 * i));
  }
}

// A server's pad data message for slot.
void BuildResponse(uint8_t (&out)[Proto::kPadDataSize], int slot) {
  std::memset(out, 0, sizeof(out));
  std::memcpy(out, "DSUS", 4);
  out[4] = uint8_t(Proto::kVersion);
  out[5] = uint8_t(Proto::kVersion >> 8);
  out[6] = uint8_t(Proto::kPadDataSize - Proto::kHeaderSize);
  PutU32(out + 16, Proto::kTypePadData);
  out[20] = uint8_t(slot);
  out[21] = 2;    // connected
  out[23] = 1;    // USB
  out[31] = 1;    // active
  out[36] = 0x18; // options, d-pad up
  out[37] = 0x22; // R2, cross
  out[38] = 1;    // home
  out[40] = 255;  // left stick right
  out[41] = 0;    // left stick down
  out[42] = 128;
  out[43] = 200;
  out[54] = 77;   // R2
  out[55] = 33;   // L2
  PutU32(out + 8, Proto::Crc32(out, sizeof(out)));
}

} // namespace

int main() {
  // The standard CRC-32 check value.
  const char* check = "123456789";
  REQUIRE(Proto::Crc32(reinterpret_cast< const uint8_t* >(check), 9) == 0xCBF43926u);

  // A pad data request for every slot: magic, version, length without the header, type.
  const std::vector< uint8_t > request = Proto::BuildRequest(0x11223344u, Proto::kTypePadData, {0, 0, 0, 0, 0, 0, 0, 0});
  REQUIRE(request.size() == 28);
  REQUIRE(std::memcmp(request.data(), "DSUC", 4) == 0);
  REQUIRE(request[4] == 0xE9 && request[5] == 0x03);
  REQUIRE(request[6] == 12 && request[7] == 0);
  REQUIRE(request[12] == 0x44 && request[15] == 0x11);
  REQUIRE(request[16] == 0x02 && request[18] == 0x10);
  {
    std::vector< uint8_t > copy = request;
    std::memset(copy.data() + 8, 0, 4);
    const uint32_t crc = Proto::Crc32(copy.data(), copy.size());
    REQUIRE(request[8] == uint8_t(crc) && request[11] == uint8_t(crc >> 24));
  }

  // A response's fields.
  uint8_t response[Proto::kPadDataSize];
  BuildResponse(response, 1);
  Proto::Pad pad;
  REQUIRE(Proto::ParsePadData(response, sizeof(response), pad));
  REQUIRE(pad.slot == 1 && pad.connected && pad.connection == 1);
  REQUIRE(pad.buttons1 == 0x18 && pad.buttons2 == 0x22 && pad.home == 1);
  REQUIRE(pad.lx == 255 && pad.ly == 0 && pad.rx == 128 && pad.ry == 200);
  REQUIRE(pad.r2 == 77 && pad.l2 == 33);

  // An inactive pad parses but isn't connected.
  BuildResponse(response, 0);
  response[31] = 0;
  std::memset(response + 8, 0, 4);
  PutU32(response + 8, Proto::Crc32(response, sizeof(response)));
  REQUIRE(Proto::ParsePadData(response, sizeof(response), pad) && !pad.connected);

  // Rejects: a short packet, a bad CRC, a client's magic, a newer protocol.
  BuildResponse(response, 1);
  REQUIRE(!Proto::ParsePadData(response, sizeof(response) - 1, pad));
  response[40] ^= 1;
  REQUIRE(!Proto::ParsePadData(response, sizeof(response), pad));
  BuildResponse(response, 1);
  response[3] = 'C';
  REQUIRE(!Proto::ParsePadData(response, sizeof(response), pad));
  BuildResponse(response, 1);
  response[4] = 0xEA;
  REQUIRE(!Proto::ParsePadData(response, sizeof(response), pad));

  // Slot info, which keeps an idle pad alive.
  {
    uint8_t info[Proto::kPortInfoSize] = {};
    std::memcpy(info, "DSUS", 4);
    info[4] = uint8_t(Proto::kVersion);
    info[5] = uint8_t(Proto::kVersion >> 8);
    info[6] = uint8_t(Proto::kPortInfoSize - Proto::kHeaderSize);
    PutU32(info + 16, Proto::kTypePorts);
    info[20] = 2;
    info[21] = 2;
    PutU32(info + 8, Proto::Crc32(info, sizeof(info)));
    int slot = -1;
    bool connected = false;
    REQUIRE(Proto::ParsePortInfo(info, sizeof(info), slot, connected) && slot == 2 && connected);
    REQUIRE(!Proto::ParsePadData(info, sizeof(info), pad));
    REQUIRE(!Proto::ParsePortInfo(response, sizeof(response), slot, connected));
  }

  // Server addresses.
  std::string host;
  std::string port;
  REQUIRE(SplitServer("127.0.0.1:26760", host, port) && host == "127.0.0.1" && port == "26760");
  REQUIRE(SplitServer(" 192.168.1.20 ", host, port) && host == "192.168.1.20" && port == "26760");
  REQUIRE(SplitServer("deck.local:26761", host, port) && host == "deck.local" && port == "26761");
  REQUIRE(SplitServer("[::1]:26762", host, port) && host == "::1" && port == "26762");
  REQUIRE(SplitServer("fe80::1", host, port) && host == "fe80::1" && port == "26760");
  REQUIRE(!SplitServer("   ", host, port));
  REQUIRE(!SplitServer(":26760", host, port));

  return port_test::Finish();
}
