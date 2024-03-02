#include <WiFi.h>
#include "packetiser.hpp"

const char* ssid     = "Hexapod_v2";
const char* password = "testing_testing123";

WiFiServer portListener(16523);
Packetiser::Packetiser packetiser;

void setup()
{
  Serial.begin(115200);
  Serial.println("\n[*] Creating AP");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  Serial.print("[+] AP Created with IP Gateway ");
  Serial.println(WiFi.softAPIP());

  portListener.begin();
}

void loop()
{
  
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