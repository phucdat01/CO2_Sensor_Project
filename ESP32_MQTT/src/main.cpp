#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// Thông tin WiFi
const char *ssid = "Free";
const char *password = "abc@1234";

// Thông tin ThingsBoard MQTT
const char* mqttServer = "thingsboard.cloud";
const int mqttPort = 1883;
const char* mqttUser = "gztP3Pgl5R2ZkGPv6UB5"; // Access Token
const char* mqttTopic = "v1/devices/me/telemetry";

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32_Client", mqttUser, NULL)) {
    } else {
      delay(5000);
    }
  }
}

void setup() {
  Serial2.begin(9600); // UART2 để nhận dữ liệu từ STM32
  setup_wifi();
  client.setServer(mqttServer, mqttPort);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Đọc dữ liệu từ STM32 qua UART2
  if (Serial2.available()) {
    String data = Serial2.readStringUntil('\n');
    data.trim(); // Loại bỏ ký tự thừa
    float co2 = data.toFloat(); // Chuyển dữ liệu thành số thực
    // Tạo payload JSON cho ThingsBoard
    String payload = "{\"co2\":" + String(co2, 0) + "}"; // Định dạng JSON: {"co2":123}
    // Gửi dữ liệu qua MQTT
    client.publish(mqttTopic, payload.c_str());
  }
}