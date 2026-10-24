// DSU controller client (port_dsu.h). The network thread asks the server for the
// slots' info and pad data every second (a server stops sending after about five
// seconds without a request) and keeps the newest packet per slot. Update(), on
// the game thread, turns each live slot into an SDL virtual gamepad.

#include "port_dsu.h"

#include "port_input_settings.h"
#include "port_log.h"

#include <SDL3/SDL.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <mutex>
#include <random>
#include <thread>
#include <utility>

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
constexpr auto kRequestEvery = std::chrono::seconds(1);
// Nothing about a slot for this long: the pad is unplugged (its virtual pad goes
// away). Some servers send pad data only on a change, so slot info counts too.
constexpr auto kSlotTimeout = std::chrono::seconds(3);
constexpr auto kServerTimeout = std::chrono::seconds(3);
constexpr int kPollMs = 100;

struct Slot {
  bool valid = false;
  Clock::time_point when;
  Proto::Pad pad;
  unsigned packets = 0;       // this second
  unsigned packetsShown = 0;  // the last whole second
  Clock::time_point secondStart;
};

class Client {
public:
  ~Client() { Stop(); }

  void Use(const std::string& server) {
    if (mRunning && server == mServer) {
      return;
    }
    Stop();
    mServer = server;
    {
      std::lock_guard< std::mutex > lock(mMutex);
      mSlots = {};
      mLastValid = {};
      mError.clear();
    }
    mStop = false;
    mRunning = true;
    mThread = std::thread([this, server] { Run(server); });
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
    mSlots = {};
  }

  bool Running() const { return mRunning; }

  // The live slots' pads (connected, heard from lately).
  std::array< std::pair< bool, Proto::Pad >, kSlotCount > Live() {
    std::array< std::pair< bool, Proto::Pad >, kSlotCount > out{};
    std::lock_guard< std::mutex > lock(mMutex);
    const Clock::time_point now = Clock::now();
    for (int i = 0; i < kSlotCount; ++i) {
      out[i] = {mSlots[i].valid && now - mSlots[i].when < kSlotTimeout, mSlots[i].pad};
    }
    return out;
  }

  std::string ServerStatus() {
    if (!mRunning) {
      return "off";
    }
    std::lock_guard< std::mutex > lock(mMutex);
    if (!mError.empty()) {
      return mServer + ": " + mError;
    }
    if (mLastValid == Clock::time_point{} || Clock::now() - mLastValid > kServerTimeout) {
      return mServer + ": waiting for the server...";
    }
    return mServer + ": connected";
  }

  std::vector< std::string > SlotStatus() {
    std::vector< std::string > out;
    std::lock_guard< std::mutex > lock(mMutex);
    const Clock::time_point now = Clock::now();
    for (int i = 0; i < kSlotCount; ++i) {
      const Slot& s = mSlots[i];
      if (!s.valid || now - s.when >= kSlotTimeout) {
        continue;
      }
      const char* connection = s.pad.connection == 1 ? "USB" : s.pad.connection == 2 ? "Bluetooth" : "network";
      out.push_back("DSU slot " + std::to_string(i + 1) + ": " + connection + ", " + std::to_string(s.packetsShown) +
                    " packets/s");
    }
    return out;
  }

private:
  void SetError(std::string error) {
    std::lock_guard< std::mutex > lock(mMutex);
    mError = std::move(error);
  }

  void Run(std::string server) {
    std::string host;
    std::string port;
    if (!SplitServer(server, host, port)) {
      SetError("not a host:port");
      return;
    }
    if (!InitSockets()) {
      SetError("sockets unavailable");
      return;
    }
    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    addrinfo* found = nullptr;
    // Retried: a DNS name may only resolve once the network is up.
    while (!mStop && getaddrinfo(host.c_str(), port.c_str(), &hints, &found) != 0) {
      SetError("can't resolve " + host);
      for (int i = 0; i < 20 && !mStop; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
      }
    }
    if (mStop) {
      return;
    }
    sockaddr_storage addr{};
    const socklen_t addrLen = socklen_t(found->ai_addrlen);
    std::memcpy(&addr, found->ai_addr, found->ai_addrlen);
    const Socket s = socket(found->ai_family, SOCK_DGRAM, IPPROTO_UDP);
    freeaddrinfo(found);
    if (s == kNoSocket) {
      SetError("can't open a UDP socket");
      return;
    }
    SetError("");
    PortLog::Write("[dsu] client for %s\n", server.c_str());

    const uint32_t clientId = std::random_device{}();
    // The four slots' info, and pad data for every slot (registration flags 0).
    const std::vector< uint8_t > ports = Proto::BuildRequest(clientId, Proto::kTypePorts, {kSlotCount, 0, 0, 0, 0, 1, 2, 3});
    const std::vector< uint8_t > data = Proto::BuildRequest(clientId, Proto::kTypePadData, {0, 0, 0, 0, 0, 0, 0, 0});
    Clock::time_point nextRequest{};
    uint8_t buffer[2048];
    while (!mStop) {
      const Clock::time_point now = Clock::now();
      if (now >= nextRequest) {
        for (const auto* p : {&ports, &data}) {
          sendto(s, reinterpret_cast< const char* >(p->data()), int(p->size()), 0,
                 reinterpret_cast< const sockaddr* >(&addr), addrLen);
        }
        nextRequest = now + kRequestEvery;
      }
      pollfd fd{};
      fd.fd = s;
      fd.events = POLLIN;
      if (PollSocket(&fd, kPollMs) <= 0 || (fd.revents & POLLIN) == 0) {
        continue;
      }
      // Errors (an ICMP unreachable while no server listens) wait for the next request.
      const int got = int(recv(s, reinterpret_cast< char* >(buffer), int(sizeof(buffer)), 0));
      if (got <= 0) {
        continue;
      }
      int infoSlot = 0;
      bool infoConnected = false;
      if (Proto::ParsePortInfo(buffer, std::size_t(got), infoSlot, infoConnected)) {
        if (infoSlot >= 0 && infoSlot < kSlotCount) {
          std::lock_guard< std::mutex > lock(mMutex);
          mLastValid = Clock::now();
          Slot& slot = mSlots[infoSlot];
          // Only keeps a pad already seen alive: its buttons come with pad data.
          if (!infoConnected) {
            slot.valid = false;
          } else if (slot.valid) {
            slot.when = mLastValid;
          }
        }
        continue;
      }
      Proto::Pad pad;
      if (!Proto::ParsePadData(buffer, std::size_t(got), pad) || pad.slot < 0 || pad.slot >= kSlotCount) {
        continue;
      }
      std::lock_guard< std::mutex > lock(mMutex);
      const Clock::time_point when = Clock::now();
      mLastValid = when;
      Slot& slot = mSlots[pad.slot];
      slot.valid = pad.connected;
      if (!pad.connected) {
        continue;
      }
      slot.pad = pad;
      slot.when = when;
      if (when - slot.secondStart >= std::chrono::seconds(1)) {
        slot.packetsShown = slot.packets;
        slot.packets = 0;
        slot.secondStart = when;
      }
      ++slot.packets;
    }
    CloseSocket(s);
  }

  // The game thread alone starts and stops the client.
  std::thread mThread;
  std::atomic< bool > mStop{false};
  bool mRunning = false;
  std::string mServer;
  // Shared with the thread.
  std::mutex mMutex;
  std::array< Slot, kSlotCount > mSlots{};
  Clock::time_point mLastValid{};
  std::string mError;
};

Client& TheClient() {
  static Client client;
  return client;
}

// ---- Virtual gamepads ----

struct Virtual {
  SDL_JoystickID id = 0;
  SDL_Joystick* joy = nullptr;
  int layout = kLayoutGameCube;
};
std::array< Virtual, kSlotCount > sVirtual{};

Virtual Attach(int slot, int layout) {
  const std::string name = "DSU slot " + std::to_string(slot + 1);
  SDL_VirtualJoystickDesc desc;
  SDL_INIT_INTERFACE(&desc);
  desc.type = SDL_JOYSTICK_TYPE_GAMEPAD;
  // The ids decide SDL's gamepad type and with it Aurora's default mapping:
  // Nintendo's GameCube adapter (GameCube mapping, L/R clicks on MISC3/MISC4, no
  // trigger emulation), or a DualShock 4. The name marks it as a real controller
  // rather than the touch overlay (PADIsTouchOverlayJoystick).
  const bool gc = layout == kLayoutGameCube;
  desc.vendor_id = gc ? 0x057E : 0x054C;
  desc.product_id = gc ? 0x0337 : 0x09CC;
  desc.naxes = SDL_GAMEPAD_AXIS_COUNT;
  desc.nbuttons = SDL_GAMEPAD_BUTTON_COUNT;
  desc.button_mask = (1u << SDL_GAMEPAD_BUTTON_COUNT) - 1;
  desc.axis_mask = (1u << SDL_GAMEPAD_AXIS_COUNT) - 1;
  desc.name = name.c_str();
  Virtual v;
  v.layout = layout;
  v.id = SDL_AttachVirtualJoystick(&desc);
  v.joy = v.id != 0 ? SDL_OpenJoystick(v.id) : nullptr;
  if (v.joy != nullptr) {
    for (const int t : {SDL_GAMEPAD_AXIS_LEFT_TRIGGER, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER}) {
      SDL_SetJoystickVirtualAxis(v.joy, t, SDL_JOYSTICK_AXIS_MIN); // released
    }
    PortLog::Write("[dsu] %s connected (%s layout)\n", name.c_str(), gc ? "GameCube" : "PlayStation");
  } else {
    PortLog::Write("[dsu] %s: no virtual gamepad: %s\n", name.c_str(), SDL_GetError());
    if (v.id != 0) {
      SDL_DetachVirtualJoystick(v.id);
      v.id = 0;
    }
  }
  return v;
}

void Detach(int slot) {
  Virtual& v = sVirtual[slot];
  if (v.joy != nullptr) {
    SDL_CloseJoystick(v.joy);
  }
  if (v.id != 0) {
    SDL_DetachVirtualJoystick(v.id);
    PortLog::Write("[dsu] DSU slot %d disconnected\n", slot + 1);
  }
  v = Virtual{};
}

// DSU stick byte (128 centre, larger = right / up) to an SDL axis (negative = left / up).
Sint16 StickAxis(uint8_t v, bool flipY) {
  const int c = (int(v) - 128) * 256;
  return Sint16(std::clamp(flipY ? -c : c, -32768, 32767));
}

// DSU trigger byte (0-255) to a trigger axis (-32768 released .. 32767 pulled).
Sint16 TriggerAxis(uint8_t v) { return Sint16(-32768 + int(v) * 257); }

void Apply(SDL_Joystick* j, const Proto::Pad& d, int layout) {
  const uint8_t b1 = d.buttons1;
  const uint8_t b2 = d.buttons2;
  // From bit 0 up, buttons1: share, L3, R3, options, up, right, down, left;
  // buttons2: L2, R2, L1, R1, square, cross, circle, triangle.
  if (layout == kLayoutGameCube) {
    // SDL's GameCube buttons: A south, B west, X east, Y north, Z right shoulder,
    // the L/R clicks MISC3/MISC4. The server sends B as circle, X as square, Z as
    // R3 and the clicks as L1/R1 (or only L2/R2's bits).
    const std::pair< int, bool > buttons[] = {
        {SDL_GAMEPAD_BUTTON_SOUTH, (b2 & 0x20) != 0},          {SDL_GAMEPAD_BUTTON_WEST, (b2 & 0x40) != 0},
        {SDL_GAMEPAD_BUTTON_EAST, (b2 & 0x10) != 0},           {SDL_GAMEPAD_BUTTON_NORTH, (b2 & 0x80) != 0},
        {SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER, (b1 & 0x04) != 0}, {SDL_GAMEPAD_BUTTON_MISC3, (b2 & 0x05) != 0},
        {SDL_GAMEPAD_BUTTON_MISC4, (b2 & 0x0A) != 0},          {SDL_GAMEPAD_BUTTON_START, (b1 & 0x08) != 0},
        {SDL_GAMEPAD_BUTTON_BACK, (b1 & 0x01) != 0},           {SDL_GAMEPAD_BUTTON_LEFT_STICK, (b1 & 0x02) != 0},
        {SDL_GAMEPAD_BUTTON_DPAD_UP, (b1 & 0x10) != 0},        {SDL_GAMEPAD_BUTTON_DPAD_RIGHT, (b1 & 0x20) != 0},
        {SDL_GAMEPAD_BUTTON_DPAD_DOWN, (b1 & 0x40) != 0},      {SDL_GAMEPAD_BUTTON_DPAD_LEFT, (b1 & 0x80) != 0},
        {SDL_GAMEPAD_BUTTON_GUIDE, d.home != 0},
    };
    for (const auto& [b, down] : buttons) {
      SDL_SetJoystickVirtualButton(j, b, down);
    }
  } else {
    const std::pair< int, bool > buttons[] = {
        {SDL_GAMEPAD_BUTTON_BACK, (b1 & 0x01) != 0},          {SDL_GAMEPAD_BUTTON_LEFT_STICK, (b1 & 0x02) != 0},
        {SDL_GAMEPAD_BUTTON_RIGHT_STICK, (b1 & 0x04) != 0},   {SDL_GAMEPAD_BUTTON_START, (b1 & 0x08) != 0},
        {SDL_GAMEPAD_BUTTON_DPAD_UP, (b1 & 0x10) != 0},       {SDL_GAMEPAD_BUTTON_DPAD_RIGHT, (b1 & 0x20) != 0},
        {SDL_GAMEPAD_BUTTON_DPAD_DOWN, (b1 & 0x40) != 0},     {SDL_GAMEPAD_BUTTON_DPAD_LEFT, (b1 & 0x80) != 0},
        {SDL_GAMEPAD_BUTTON_LEFT_SHOULDER, (b2 & 0x04) != 0}, {SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER, (b2 & 0x08) != 0},
        {SDL_GAMEPAD_BUTTON_WEST, (b2 & 0x10) != 0},          {SDL_GAMEPAD_BUTTON_SOUTH, (b2 & 0x20) != 0},
        {SDL_GAMEPAD_BUTTON_EAST, (b2 & 0x40) != 0},          {SDL_GAMEPAD_BUTTON_NORTH, (b2 & 0x80) != 0},
        {SDL_GAMEPAD_BUTTON_GUIDE, d.home != 0},              {SDL_GAMEPAD_BUTTON_TOUCHPAD, d.touch != 0},
    };
    for (const auto& [b, down] : buttons) {
      SDL_SetJoystickVirtualButton(j, b, down);
    }
  }
  SDL_SetJoystickVirtualAxis(j, SDL_GAMEPAD_AXIS_LEFTX, StickAxis(d.lx, false));
  SDL_SetJoystickVirtualAxis(j, SDL_GAMEPAD_AXIS_LEFTY, StickAxis(d.ly, true));
  SDL_SetJoystickVirtualAxis(j, SDL_GAMEPAD_AXIS_RIGHTX, StickAxis(d.rx, false));
  SDL_SetJoystickVirtualAxis(j, SDL_GAMEPAD_AXIS_RIGHTY, StickAxis(d.ry, true));
  // A server that only sets the L2/R2 bits still gives a full pull.
  SDL_SetJoystickVirtualAxis(j, SDL_GAMEPAD_AXIS_LEFT_TRIGGER, TriggerAxis((b2 & 0x01) != 0 ? 255 : d.l2));
  SDL_SetJoystickVirtualAxis(j, SDL_GAMEPAD_AXIS_RIGHT_TRIGGER, TriggerAxis((b2 & 0x02) != 0 ? 255 : d.r2));
}

} // namespace

void Update() {
  const PortInput::Settings& s = PortInput::Get();
  Client& client = TheClient();
  if (s.dsuEnabled) {
    client.Use(s.dsuServer);
  } else {
    client.Stop();
  }
  const auto live = client.Live();
  for (int slot = 0; slot < kSlotCount; ++slot) {
    Virtual& v = sVirtual[slot];
    if (!live[slot].first) {
      if (v.id != 0) {
        Detach(slot);
      }
      continue;
    }
    // A layout change plugs the pad in again as the other type.
    if (v.id != 0 && v.layout != s.dsuLayout) {
      Detach(slot);
    }
    if (v.id == 0) {
      v = Attach(slot, s.dsuLayout);
    }
    if (v.joy != nullptr) {
      Apply(v.joy, live[slot].second, v.layout);
    }
  }
}

std::string ServerStatus() { return TheClient().ServerStatus(); }

std::vector< std::string > SlotStatus() { return TheClient().SlotStatus(); }

} // namespace PortDsu
