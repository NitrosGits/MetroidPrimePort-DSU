#include "port_dsu.h"
#include "port_test.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

#if !defined(_WIN32)
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace {

using namespace PortDsu;

void PutU32(uint8_t* p, uint32_t v) {
  for (int i = 0; i < 4; ++i) {
    p[i] = uint8_t(v >> (8 * i));
  }
}

void PutF32(uint8_t* p, float f) {
  uint32_t bits;
  std::memcpy(&bits, &f, sizeof(bits));
  PutU32(p, bits);
}

// A server's pad data message for slot, with accel in g and gyro in deg/s.
void BuildResponse(uint8_t (&out)[Proto::kPadDataResponseSize], int slot, const float accel[3], const float gyro[3]) {
  std::memset(out, 0, sizeof(out));
  std::memcpy(out, "DSUS", 4);
  out[4] = uint8_t(Proto::kVersion);
  out[5] = uint8_t(Proto::kVersion >> 8);
  out[6] = uint8_t(Proto::kPadDataResponseSize - Proto::kHeaderSize);
  PutU32(out + 16, Proto::kTypePadData);
  out[20] = uint8_t(slot);
  out[21] = 2; // connected
  out[31] = 1;
  PutU32(out + 68, 123456);
  for (int i = 0; i < 3; ++i) {
    PutF32(out + 76 + 4 * i, accel[i]);
    PutF32(out + 88 + 4 * i, gyro[i]);
  }
  PutU32(out + 8, Proto::Crc32(out, sizeof(out)));
}

bool Near(float a, float b) { return std::fabs(a - b) < 1e-4f; }

} // namespace

int main() {
  // The standard CRC-32 check value.
  const char* check = "123456789";
  REQUIRE(Proto::Crc32(reinterpret_cast< const uint8_t* >(check), 9) == 0xCBF43926u);

  // The request: magic, version, length without the header, type, register by slot.
  uint8_t request[Proto::kPadDataRequestSize];
  Proto::BuildPadDataRequest(request, 0x11223344u, 2);
  REQUIRE(std::memcmp(request, "DSUC", 4) == 0);
  REQUIRE(request[4] == 0xE9 && request[5] == 0x03);
  REQUIRE(request[6] == 12 && request[7] == 0);
  REQUIRE(request[12] == 0x44 && request[15] == 0x11);
  REQUIRE(request[16] == 0x02 && request[18] == 0x10);
  REQUIRE(request[20] == 1 && request[21] == 2);
  {
    uint8_t copy[sizeof(request)];
    std::memcpy(copy, request, sizeof(copy));
    std::memset(copy + 8, 0, 4);
    const uint32_t crc = Proto::Crc32(copy, sizeof(copy));
    REQUIRE(request[8] == uint8_t(crc) && request[11] == uint8_t(crc >> 24));
  }

  // A response turns into SDL's axes: a flat pad at rest reads +1 g up, and
  // pitch up / yaw right / roll right become +x / -y / -z.
  const float accel[3] = {0.f, -1.f, 0.f};
  const float gyro[3] = {90.f, 45.f, -30.f};
  uint8_t response[Proto::kPadDataResponseSize];
  BuildResponse(response, 1, accel, gyro);
  Proto::Motion motion;
  REQUIRE(Proto::ParsePadData(response, sizeof(response), motion));
  REQUIRE(motion.slot == 1 && motion.connected && motion.timestampUs == 123456);
  REQUIRE(Near(motion.accel[0], 0.f) && Near(motion.accel[1], 9.80665f) && Near(motion.accel[2], 0.f));
  REQUIRE(Near(motion.gyro[0], 1.5707964f));
  REQUIRE(Near(motion.gyro[1], -0.7853982f));
  REQUIRE(Near(motion.gyro[2], 0.5235988f));

  // Rejects: a short packet, a bad CRC, a client's magic.
  REQUIRE(!Proto::ParsePadData(response, sizeof(response) - 1, motion));
  response[90] ^= 1;
  REQUIRE(!Proto::ParsePadData(response, sizeof(response), motion));
  BuildResponse(response, 1, accel, gyro);
  response[3] = 'C';
  REQUIRE(!Proto::ParsePadData(response, sizeof(response), motion));

#if !defined(_WIN32)
  // The client against a loopback server: it registers its slot, and what the
  // server sends back comes out of Read.
  const int server = socket(AF_INET, SOCK_DGRAM, 0);
  REQUIRE(server >= 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  REQUIRE(bind(server, reinterpret_cast< sockaddr* >(&addr), sizeof(addr)) == 0);
  socklen_t len = sizeof(addr);
  REQUIRE(getsockname(server, reinterpret_cast< sockaddr* >(&addr), &len) == 0);
  const int port = ntohs(addr.sin_port);

  float outGyro[3];
  float outAccel[3];
  REQUIRE(!Read(outGyro, outAccel));
  Use("127.0.0.1", port, 1);

  pollfd fd{server, POLLIN, 0};
  REQUIRE(poll(&fd, 1, 3000) == 1);
  uint8_t got[64];
  sockaddr_in client{};
  socklen_t clientLen = sizeof(client);
  const ssize_t n = recvfrom(server, got, sizeof(got), 0, reinterpret_cast< sockaddr* >(&client), &clientLen);
  REQUIRE(n == ssize_t(Proto::kPadDataRequestSize));
  REQUIRE(std::memcmp(got, "DSUC", 4) == 0 && got[20] == 1 && got[21] == 1);

  // Another slot's data is ignored.
  BuildResponse(response, 0, accel, gyro);
  sendto(server, response, sizeof(response), 0, reinterpret_cast< sockaddr* >(&client), clientLen);
  BuildResponse(response, 1, accel, gyro);
  sendto(server, response, sizeof(response), 0, reinterpret_cast< sockaddr* >(&client), clientLen);
  bool read = false;
  for (int i = 0; i < 200 && !read; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    read = Read(outGyro, outAccel);
  }
  REQUIRE(read);
  REQUIRE(Near(outGyro[0], 1.5707964f) && Near(outAccel[1], 9.80665f));
  REQUIRE(Status().find("receiving") == 0);

  Stop();
  REQUIRE(!Read(outGyro, outAccel));
  REQUIRE(Status() == "off");
  close(server);
#endif

  return port_test::Finish();
}
