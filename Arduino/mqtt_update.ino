#include <WiFi.h>
#include <PubSubClient.h>

const char* WIFI_SSID = "Aml Office 4th";
const char* WIFI_PASSWORD = "Aml@2026";
const char* MQTT_HOST = "192.168.110.8";  // Computer/server running Mosquitto
const int MQTT_PORT = 1883;

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

void connectMqtt() {
  while (!mqtt.connected()) {
    mqtt.connect("agv-r1-esp32");
    delay(1000);
  }
}

void reportRfid(const char* rfidId) {
  mqtt.publish("agv/R1/node", rfidId);
  // Or:
  // mqtt.publish("agv/R1/node", "{\"node_id\":\"Gate_1\"}");
}

void setup() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  connectMqtt();

  reportRfid("PUT_YOUR_RFID_UID_HERE");
}

void loop() {
  if (!mqtt.connected()) connectMqtt();
  mqtt.loop();

  // After reading a tag, publish the UID exactly as the reader returns it.
  // Map that UID to a warehouse node in backend/rfid_node_map.json.
  // Example: reportRfid("A1B2C3D4");
  delay(1000);
}
