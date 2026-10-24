#include "port_dsu.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <random>
#include <thread>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <netdb.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif

namespace PortDsu {
namespace Proto {
namespace {

constexpr float kRadPerDeg = 0.017453292519943295f;
constexpr float kStandardGravity = 9.80665f;
constexpr uint8_t kRegisterPadId = 1;

void PutU16(uint8_t* p, uint16_t v) {
  p[0] = uint8_t(v);
  p[1] = uint8_t(v >> 8);
}

void PutU32(uint8_t* p, uint32_t v) {
  for (int i = 0; i < 4; ++i) {
    p[i] = uint8_t(v >> (8 * i));
  }
}

uint32_t GetU32(const uint8_t* p) { return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24; }

uint64_t GetU64(const uint8_t* p) { return uint64_t(GetU32(p)) | uint64_t(GetU32(p + 4)) << 32; }

float GetF32(const uint8_t* p) {
  const uint32_t bits = GetU32(p);
  float f;
  std::memcpy(&f, &bits, sizeof(f));
  return f;
}

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

void BuildPadDataRequest(uint8_t (&out)[kPadDataRequestSize], uint32_t clientId, int slot) {
  std::memset(out, 0, sizeof(out));
  std::memcpy(out, "DSUC", 4);
  PutU16(out + 4, kVersion);
  // The length counts the message type and what follows, not the header.
  PutU16(out + 6, uint16_t(kPadDataRequestSize - kHeaderSize));
  PutU32(out + 12, clientId);
  PutU32(out + 16, kTypePadData);
  out[20] = kRegisterPadId;
  out[21] = uint8_t(slot);
  PutU32(out + 8, Crc32(out, sizeof(out)));
}

bool ParsePadData(const uint8_t* data, std::size_t size, Motion& out) {
  if (size < kPadDataResponseSize || std::memcmp(data, "DSUS", 4) != 0 || GetU32(data + 16) != kTypePadData) {
    return false;
  }
  // The CRC covers the whole message (a newer server's extra fields too) with
  // its own field zeroed.
  const std::size_t length = kHeaderSize + (uint16_t(data[6]) | uint16_t(data[7]) << 8);
  uint8_t packet[1024];
  if (length > size || length < kPadDataResponseSize || length > sizeof(packet)) {
    return false;
  }
  std::memcpy(packet, data, length);
  std::memset(packet + 8, 0, 4);
  if (Crc32(packet, length) != GetU32(data + 8)) {
    return false;
  }
  out.slot = data[20];
  out.connected = data[21] == 2; // pad_state: 0 disconnected, 1 reserved, 2 connected
  out.timestampUs = GetU64(data + 68);
  // DSU's accel is in g with x left, y down and z away from the player (as
  // Dolphin reads it), its gyro pitch up, yaw right and roll right in deg/s;
  // SDL counts x right, y up, z toward the player, counter-clockwise positive.
  out.accel[0] = -GetF32(data + 76) * kStandardGravity;
  out.accel[1] = -GetF32(data + 80) * kStandardGravity;
  out.accel[2] = -GetF32(data + 84) * kStandardGravity;
  out.gyro[0] = GetF32(data + 88) * kRadPerDeg;
  out.gyro[1] = -GetF32(data + 92) * kRadPerDeg;
  out.gyro[2] = -GetF32(data + 96) * kRadPerDeg;
  return true;
}

} // namespace Proto

namespace {

#if defined(_WIN32)
using Socket = SOCKET;
const Socket kNoSocket = INVALID_SOCKET;
void CloseSocket(Socket s) { closesocket(s); }
int PollSocket(pollfd* fd, int timeoutMs) { return WSAPoll(fd, 1, timeoutMs); }
bool InitSockets() {
  static const bool ok = [] {
    WSADATA data{};
    return WSAStartup(MAKEWORD(2, 2), &data) == 0;
  }();
  return ok;
}
#else
using Socket = int;
const Socket kNoSocket = -1;
void CloseSocket(Socket s) { close(s); }
int PollSocket(pollfd* fd, int timeoutMs) { return poll(fd, 1, timeoutMs); }
bool InitSockets() { return true; }
#endif

using Clock = std::chrono::steady_clock;
// Servers drop a client that has not asked for about five seconds.
constexpr auto kReregister = std::chrono::seconds(1);
constexpr auto kStale = std::chrono::seconds(1);
constexpr int kPollMs = 250;

class Client {
public:
  ~Client() { Stop(); }

  void Use(const std::string& host, int port, int slot) {
    if (mRunning && host == mHost && port == mPort && slot == mSlot) {
      return;
    }
    Stop();
    mHost = host;
    mPort = port;
    mSlot = slot;
    {
      std::lock_guard< std::mutex > lock(mMutex);
      mHaveSample = false;
      mGyroCount = 0;
      mStatus = "connecting to " + Address();
    }
    mStop = false;
    mRunning = true;
    mThread = std::thread([this, host, port, slot] { Run(host, port, slot); });
  }

  void Stop() {
    if (!mRunning) {
      return;
    }
    mStop = true;
    if (mThread.joinable()) {
      mThread.join();
    }
    mRunning = false;
    std::lock_guard< std::mutex > lock(mMutex);
    mHaveSample = false;
    mStatus = "off";
  }

  bool Read(float gyro[3], float accel[3]) {
    std::lock_guard< std::mutex > lock(mMutex);
    if (!mHaveSample || Clock::now() - mLastSample > kStale) {
      return false;
    }
    // A frame between two packets repeats the last rate rather than reading still.
    if (mGyroCount > 0) {
      for (int i = 0; i < 3; ++i) {
        mGyro[i] = mGyroSum[i] / float(mGyroCount);
        mGyroSum[i] = 0.f;
      }
      mGyroCount = 0;
    }
    std::memcpy(gyro, mGyro, sizeof(mGyro));
    std::memcpy(accel, mAccel, sizeof(mAccel));
    return true;
  }

  std::string Status() {
    std::lock_guard< std::mutex > lock(mMutex);
    if (mRunning && mHaveSample && Clock::now() - mLastSample > kStale) {
      return "no data from " + Address() + " slot " + std::to_string(mSlot + 1);
    }
    return mStatus;
  }

private:
  std::string Address() const { return mHost + ":" + std::to_string(mPort); }

  void SetStatus(std::string status) {
    std::lock_guard< std::mutex > lock(mMutex);
    mStatus = std::move(status);
  }

  void Run(std::string host, int port, int slot) {
    const std::string address = host + ":" + std::to_string(port);
    if (!InitSockets()) {
      SetStatus("sockets unavailable");
      return;
    }
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    addrinfo* found = nullptr;
    // Retried while the thread runs: a DNS name may only resolve once the network is up.
    while (!mStop && getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &found) != 0) {
      SetStatus("can't resolve " + host);
      for (int i = 0; i < 8 && !mStop; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
      }
    }
    if (mStop) {
      return;
    }
    sockaddr_storage server{};
    const socklen_t serverLen = socklen_t(found->ai_addrlen);
    std::memcpy(&server, found->ai_addr, found->ai_addrlen);
    const Socket s = socket(found->ai_family, SOCK_DGRAM, IPPROTO_UDP);
    freeaddrinfo(found);
    if (s == kNoSocket) {
      SetStatus("can't open a UDP socket");
      return;
    }
    const uint32_t clientId = std::random_device{}();
    uint8_t request[Proto::kPadDataRequestSize];
    Proto::BuildPadDataRequest(request, clientId, slot);
    SetStatus("waiting for " + address + " slot " + std::to_string(slot + 1));

    Clock::time_point nextRequest{};
    uint8_t buffer[1024];
    while (!mStop) {
      const Clock::time_point now = Clock::now();
      if (now >= nextRequest) {
        sendto(s, reinterpret_cast< const char* >(request), int(sizeof(request)), 0,
               reinterpret_cast< const sockaddr* >(&server), serverLen);
        nextRequest = now + kReregister;
      }
      pollfd fd{};
      fd.fd = s;
      fd.events = POLLIN;
      if (PollSocket(&fd, kPollMs) <= 0 || (fd.revents & POLLIN) == 0) {
        continue;
      }
      // Errors (an ICMP unreachable while no server listens) only wait for the next request.
      const int got = int(recv(s, reinterpret_cast< char* >(buffer), int(sizeof(buffer)), 0));
      Proto::Motion motion;
      if (got <= 0 || !Proto::ParsePadData(buffer, std::size_t(got), motion) || motion.slot != slot ||
          !motion.connected) {
        continue;
      }
      std::lock_guard< std::mutex > lock(mMutex);
      for (int i = 0; i < 3; ++i) {
        mGyroSum[i] += motion.gyro[i];
      }
      ++mGyroCount;
      std::memcpy(mAccel, motion.accel, sizeof(mAccel));
      mLastSample = Clock::now();
      mHaveSample = true;
      mStatus = "receiving from " + address + " slot " + std::to_string(slot + 1);
    }
    CloseSocket(s);
  }

  // The game thread alone starts and stops the client.
  std::thread mThread;
  std::atomic< bool > mStop{false};
  bool mRunning = false;
  std::string mHost;
  int mPort = 0;
  int mSlot = 0;
  // Shared with the thread.
  std::mutex mMutex;
  std::string mStatus = "off";
  bool mHaveSample = false;
  Clock::time_point mLastSample;
  float mGyroSum[3] = {};
  int mGyroCount = 0;
  float mGyro[3] = {};
  float mAccel[3] = {};
};

Client& TheClient() {
  static Client client;
  return client;
}

} // namespace

void Use(const std::string& host, int port, int slot) { TheClient().Use(host, port, slot); }
void Stop() { TheClient().Stop(); }
bool Read(float gyro[3], float accel[3]) { return TheClient().Read(gyro, accel); }
std::string Status() { return TheClient().Status(); }

} // namespace PortDsu
