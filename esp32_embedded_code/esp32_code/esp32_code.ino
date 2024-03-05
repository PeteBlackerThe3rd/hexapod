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

  size_t getPacketQueueSize()
  {
    std::lock_guard<std::mutex> lck(packetQueueMutex);
    return packetQueue.size();
  };

  BasePacketPtr popPacket()
  {
    std::lock_guard<std::mutex> lck(packetQueueMutex);
    BasePacketPtr packet = packetQueue[0];
    packetQueue.erase(packetQueue.begin());
    return packet;
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
      Packetiser::PacketQueue packetQueue;

      if (client) {
        Serial.println("Socket connected");

        while (client.connected()) {

          vTaskDelay(1);

          while (client.available()>0) {
            char c = client.read();
            Buffer charBuf = {c};
            commsThread->packetiser.processInput(charBuf, packetQueue, true, true);
            // Serial.print(c);
            client.write(c);
          }

          if (packetQueue.size() > 0) {
            if (packetQueue[0].second == Packetiser::PacketType::packet) {
              try {
                BasePacketPtr packet = BasePacket::deserialise(packetQueue[0].first);
                std::lock_guard<std::mutex> lck(commsThread->packetQueueMutex);
                commsThread->packetQueue.push_back(packet);
                Serial.print("Decoded: ");
                Serial.println(packet->toString().c_str());
              }
              catch (BaseException& e) {
                Serial.print("Failed to deserialise packet: ");
                Serial.println(e.what().c_str());
              }
              Serial.print("Recieved packet: ");
            }
            else
              Serial.print("Recieved garbage: ");
            
            for (uint8_t byte : packetQueue[0].first) {
              Serial.print(byte < 16 ? "0" : "");
              Serial.print(byte, HEX);
              Serial.print(" ");
            }
            Serial.println(" ");
            packetQueue.erase(packetQueue.begin());
          }

          delay(10);
        }

        client.stop();
        Serial.println("Socket disconnected");
      }
    }
  };

  WiFiServer portListener;
  Packetiser::Packetiser packetiser;

  TaskHandle_t commsThreadTask;
  
  static constexpr char* ssid = "Hexapod_v2";
  static constexpr char* password = "testing_testing123";

  // note haven't implemented the packet class hierarchy so this is void* for now
  std::mutex packetQueueMutex;
  std::vector<BasePacketPtr> packetQueue;
};

CommsThread* commsThread;

void setup()
{
  Serial.begin(115200);
  Serial.println("\n[*] Creating AP");
  commsThread = new CommsThread;
}

void loop()
{
  //Serial.println("Something to stop core#1 watchdog triggering");
  delay(100);

  while (commsThread->getPacketQueueSize() > 0) {
    BasePacketPtr packet = commsThread->popPacket();
    Serial.print("Handling: ");
    Serial.println(packet->toString().c_str());
  }
}
