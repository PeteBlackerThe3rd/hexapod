#ifndef __COMMS_THREAD_HPP__
#define __COMMS_THREAD_HPP__
#include <WiFi.h>
#include <mutex>
#include "packetiser.hpp"
#include "packet_defs.hpp"

class CommsThread
{
public:
  CommsThread();

  size_t getPacketRxQueueSize();

  BasePacketPtr popPacket();

  bool socketConected();

  bool sendPacket(BasePacketPtr packet);

private:

  static void threadEntryPoint(void* instance);

  WiFiServer portListener;
  Packetiser::Packetiser packetiser;

  TaskHandle_t commsThreadTask;
  
  static constexpr char* ssid = "Hexapod_v2";
  static constexpr char* password = "testing_testing123";

  std::mutex packetRxQueueMutex;
  std::vector<BasePacketPtr> packetRxQueue;

  std::mutex packetTxQueueMutex;
  std::vector<BasePacketPtr> packetTxQueue;

  std::mutex socketConnectedMutex;
  bool socketConnected;
};

#endif // __COMMS_THREAD_HPP__
