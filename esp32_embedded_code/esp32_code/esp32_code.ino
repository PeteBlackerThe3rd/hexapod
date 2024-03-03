#include <WiFi.h>
#include <mutex>
#include "packetiser.hpp"

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
                      CommsThread::threadEntryPoint,   /* Task function. */
                      "commsThread",     /* name of task. */
                      10000,       /* Stack size of task */
                      this,        /* parameter of the task */
                      1,           /* priority of the task */
                      &commsThreadTask,      /* Task handle to keep track of created task */
                      0);          /* pin task to core 0 */ 

    portListener.begin();
  };

  size_t getPacketQueueSize()
  {
    std::lock_guard<std::mutex> lck(packetQueueMutex);
    return packetQueue.size();
  };

  void* popPacket()
  {
    std::lock_guard<std::mutex> lck(packetQueueMutex);
    void* packet = packetQueue[0];
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
      WiFiClient client = commsThread->portListener.available();
      Packetiser::PacketQueue packetQueue;

      if (client) {
        Serial.println("Socket connected");

        while (client.connected()) {

          while (client.available()>0) {
            char c = client.read();
            Packetiser::Buffer charBuf = {c};
            commsThread->packetiser.processInput(charBuf, packetQueue, true, true);
            // Serial.print(c);
            client.write(c);
          }

          if (packetQueue.size() > 0) {
            if (packetQueue[0].second == Packetiser::PacketType::packet)
              Serial.print("Recieved packet: ");
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
  std::vector<void*> packetQueue;
};

CommsThread* commsThread;

void setup()
{
  Serial.begin(115200);
  Serial.println("\n[*] Creating AP");
  commsThread = new CommsThread;
}

//Task1code: blinks an LED every 1000 ms
/*void runCommsThread( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;) {  
    WiFiClient client = portListener.available();
    Packetiser::PacketQueue packetQueue;

    if (client) {
      Serial.println("Socket connected");

      while (client.connected()) {

        while (client.available()>0) {
          char c = client.read();
          Packetiser::Buffer charBuf = {c};
          packetiser.processInput(charBuf, packetQueue, true, true);
          // Serial.print(c);
          client.write(c);
        }

        if (packetQueue.size() > 0) {
          if (packetQueue[0].second == Packetiser::PacketType::packet)
            Serial.print("Recieved packet: ");
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
}*/

void loop()
{
  /*WiFiClient client = portListener.available();
  Packetiser::PacketQueue packetQueue;

  if (client) {
    Serial.println("Socket connected");

    while (client.connected()) {

      while (client.available()>0) {
        char c = client.read();
        Packetiser::Buffer charBuf = {c};
        packetiser.processInput(charBuf, packetQueue, true, true);
        // Serial.print(c);
        client.write(c);
      }

      if (packetQueue.size() > 0) {
        if (packetQueue[0].second == Packetiser::PacketType::packet)
          Serial.print("Recieved packet: ");
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

  }*/
}
