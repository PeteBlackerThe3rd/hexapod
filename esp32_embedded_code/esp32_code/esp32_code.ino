#include <WiFi.h>

const char* ssid     = "Hexapod_v2";
const char* password = "testing_testing123";

WiFiServer portListener(16523);

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

  if (client) {
    Serial.println("Socket connected");

    while (client.connected()) {

      while (client.available()>0) {
        char c = client.read();
        Serial.print(c);
        client.write(c);
      }

      delay(10);
    }

    client.stop();
    Serial.println("Socket disconnected");

  }
}