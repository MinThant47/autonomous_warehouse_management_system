/*
  ESP32-CAM inbound MQTT publisher

  Arduino IDE board: AI Thinker ESP32-CAM
  Libraries: WiFi (included with ESP32 board package), PubSubClient

  This sketch publishes:
    topic:   agv/R1/inbound
    payload: {"serial_code":"E0003","pickup_location":"Gate_1"}

  The camera is not initialized in this basic MQTT example. Replace the test
  serial code with the value obtained from your QR/barcode/OCR logic, then call
  reportInbound(serialCode, pickupLocation).
*/

#include <WiFi.h>
#include <PubSubClient.h>

const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
const char* MQTT_HOST = "192.168.1.100";  // IP address of the Mosquitto server
const uint16_t MQTT_PORT = 1883;

const char* ROBOT_ID = "R1";
const char* INBOUND_TOPIC = "agv/R1/inbound";

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);
bool testMessageSent = false;


void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}


void connectMqtt() {
  while (!mqtt.connected()) {
    String clientId = "agv-" + String(ROBOT_ID) + "-esp32cam";
    if (!mqtt.connect(clientId.c_str())) {
      delay(1000);
    }
  }
}


bool reportInbound(const char* serialCode, const char* pickupLocation) {
  // Item codes and map nodes in this project use only simple text. If you add
  // quotes or backslashes to either value, escape them before constructing JSON.
  String payload = "{\"serial_code\":\"" + String(serialCode)
                 + "\",\"pickup_location\":\"" + String(pickupLocation)
                 + "\"}";

  bool sent = mqtt.publish(INBOUND_TOPIC, payload.c_str());
  Serial.print("Inbound MQTT payload: ");
  Serial.println(payload);
  Serial.println(sent ? "Published" : "Publish failed");
  return sent;
}


void setup() {
  Serial.begin(115200);
  connectWiFi();
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  connectMqtt();
}


void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  if (!mqtt.connected()) {
    connectMqtt();
  }
  mqtt.loop();

  // One-time MQTT test. This avoids creating repeated duplicate IN tasks.
  // Change these values to test another registered catalog item and map node.
  if (!testMessageSent) {
    testMessageSent = reportInbound("E0003", "Gate_1");
  }

  // When camera recognition is ready, replace the test block above with:
  // reportInbound(recognizedSerialCode, "Gate_1");
  delay(100);
}
