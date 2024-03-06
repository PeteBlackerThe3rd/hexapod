#include <WiFi.h>
#include <mutex>
#include "packetiser.hpp"
#include "packet_defs.hpp"

//const char* ssid     = "Hexapod_v2";
//const char* password = "testing_testing123";

//WiFiServer portListener(16523);
//Packetiser::Packetiser packetiser;

//TaskHandle_t commsThreadTask;

class CommsThread
{
public:
  CommsThread() : portListener(WiFiServer(16523))
  {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, password);
    Serial.print("[+] AP Created with IP Gateway ");
    Serial.println(WiFi.softAPIP());

    //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore(
                      CommsThread::threadEntryPoint,   // Task function.
                      "commsThread",     // name of task.
                      10000,       // Stack size of task 
                      this,        // parameter of the task 
                      0,           // priority of the task 
                      &commsThreadTask,      // Task handle to keep track of created task 
                      0);          // pin task to core 0 

    portListener.begin();
  };

  size_t getPacketRxQueueSize()
  {
    std::lock_guard<std::mutex> lck(packetRxQueueMutex);
    return packetRxQueue.size();
  };

  BasePacketPtr popPacket()
  {
    std::lock_guard<std::mutex> lock(packetRxQueueMutex);
    BasePacketPtr packet = packetRxQueue[0];
    packetRxQueue.erase(packetRxQueue.begin());
    return packet;
  }

  bool socketConected()
  {
    //std::lock_guard<std::mutex> lock(socketConnectedMutex);
    return socketConnected;
  }

  bool sendPacket(BasePacketPtr packet)
  {
    {
      //std::lock_guard<std::mutex> lock(socketConnectedMutex);
      if (!socketConnected)
        return false;
    }

    std::lock_guard<std::mutex> lock(packetTxQueueMutex);
    packetTxQueue.push_back(packet);
    return true;
  }

private:

  static void threadEntryPoint(void* instance)
  {
    Serial.print("Task1 running on core ");
    Serial.println(xPortGetCoreID());

    CommsThread* commsThread = static_cast<CommsThread*>(instance);

    for(;;) {  

      vTaskDelay(1);
      WiFiClient client = commsThread->portListener.available();
      Packetiser::PacketQueue rawPacketQueue;

      if (client) {
        {
          //std::lock_guard<std::mutex> lock(socketConnectedMutex);
          commsThread->socketConnected = true;
        }
        Serial.println("Socket connected");

        while (client.connected()) {

          vTaskDelay(1);

          while (client.available() > 0) {
            char c = client.read();
            Buffer charBuf = {c};
            commsThread->packetiser.processInput(charBuf, rawPacketQueue, true, true);
            // Serial.print(c);
            client.write(c);
          }

          if (rawPacketQueue.size() > 0) {
            if (rawPacketQueue[0].second == Packetiser::PacketType::packet) {
              try {
                BasePacketPtr packet = BasePacket::deserialise(rawPacketQueue[0].first);
                std::lock_guard<std::mutex> lck(commsThread->packetRxQueueMutex);
                commsThread->packetRxQueue.push_back(packet);
                Serial.print("Decoded: ");
                Serial.println(packet->toString().c_str());
              }
              catch (BaseException& e) {
                Serial.print("Failed to deserialise packet: ");
                Serial.println(e.what().c_str());
                
                for (uint8_t byte : rawPacketQueue[0].first) {
                  Serial.print(byte < 16 ? "0" : "");
                  Serial.print(byte, HEX);
                  Serial.print(" ");
                }
                Serial.println(" ");
              }
              //Serial.print("Recieved packet: ");
            }
            else {
              Serial.print("Recieved garbage: ");
            
              for (uint8_t byte : rawPacketQueue[0].first) {
                Serial.print(byte < 16 ? "0" : "");
                Serial.print(byte, HEX);
                Serial.print(" ");
              }
              Serial.println(" ");
            }

            rawPacketQueue.erase(rawPacketQueue.begin());
          }

          vTaskDelay(1);

          //bool packetToSend = false;
          BasePacketPtr packetToSend;
          {
            std::lock_guard<std::mutex> lock(commsThread->packetTxQueueMutex);
            if (commsThread->packetTxQueue.size() > 0) {
              packetToSend = commsThread->packetTxQueue[0];
              commsThread->packetTxQueue.erase(commsThread->packetTxQueue.begin());
            }
          }

          // if a packet was waited to be send in the TX queue
          if (packetToSend) {
            BufferPtr rawPacket = packetToSend->serialise();
            Buffer encapsulatedPacket = commsThread->packetiser.encapsulatePacket(*rawPacket.get());
            for (uint8_t byte : encapsulatedPacket)
              client.write(byte);
          }

          //delay(10);
        }

        client.stop();
        Serial.println("Socket disconnected");
        {
          //std::lock_guard<std::mutex> lock(socketConnectedMutex);
          commsThread->socketConnected = false;
        }
      }
    }
  };

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

CommsThread* commsThread;

uint32_t lastHKTMTime = 0;
uint16_t lastHKSeqId = 0;
uint32_t lastMotorTMTime = 0;

void setup()
{
  Serial.begin(115200);
  Serial.println("\n[*] Creating AP");
  commsThread = new CommsThread;
}

void loop()
{
  //Serial.println("Something to stop core#1 watchdog triggering");
  delay(10);

  while (commsThread->getPacketRxQueueSize() > 0) {
    BasePacketPtr packet = commsThread->popPacket();
    Serial.print("Handling: ");
    Serial.println(packet->toString().c_str());
  }

  // if the socket is connected then check if telemetry needs sending
  if (commsThread->socketConected()) {
    
    uint32_t timeNow = millis();
    
    // if more than 1 second has passed since the last 
    if (timeNow - lastHKTMTime > 100) {
      lastHKTMTime = timeNow;

      auto hkPacket = new TMHouseKeepingPacket(++lastHKSeqId);
      hkPacket->socketConnectionCount = 5;
      hkPacket->motorsOn = 1;
      hkPacket->batteryVoltage100thsVolt = 685; 

      commsThread->sendPacket(BasePacketPtr(hkPacket));
    }
  }
}
