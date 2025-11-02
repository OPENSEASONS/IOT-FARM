#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SPI.h>  // Added SPI library for ESP8266

// WiFi and MQTT Configuration (placeholders)
const char* ssid = "your-ssid";
const char* password = "your-wifi-password";
const char* mqtt_server = "your-mqtt-server";
const int mqtt_port = 1883;
const char* mqtt_user = "your-username";
const char* mqtt_pass = "your-password";
const char* routing_topic = "lora_mesh/routing_data";
const char* sensor_topic = "lora_mesh/sensor_data";

WiFiClient espClient;
PubSubClient client(espClient);

#define SPI_CS 15  // Adjust based on your dipswitch setup
#define SPI_MISO 12
#define SPI_MOSI 13
#define SPI_SCK 14

#define SPI_START_CMD 0xA5
#define SPI_ACK_CMD 0x06
#define SPI_NODATA_CMD 0x15

void setupWiFi() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
  while (!client.connected()) {
    String clientId = "LoRaMeshGateway-";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      client.subscribe("lora_mesh/control");
    } else {
      delay(2000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  for (int i = 0; i < length; i++) Serial.print((char)payload[i]);
  Serial.println();
}

void setup() {
  setupWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  SPI.begin();  // Initialize SPI
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  Serial.println("ESP8266 Gateway Ready - Waiting for ATmega data...");
}

String readFromAtmega() {
  String result = "";
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(10);
  byte response = SPI.transfer(SPI_START_CMD);
  if (response == SPI_ACK_CMD) {
    for (int i = 0; i < 512; i++) {
      byte data = SPI.transfer(0x00);
      if (data == 0) break;
      result += (char)data;
      delayMicroseconds(50);
    }
  } else if (response == SPI_NODATA_CMD) {
    result = "";
  }
  digitalWrite(SPI_CS, HIGH);
  return result;
}

void processAndPublishData(String rawData) {
  if (rawData.length() == 0) return;
  Serial.println("Raw data from ATmega: " + rawData);
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, rawData);
  if (error) {
    Serial.println(error.c_str());
    return;
  }
  if (doc.containsKey("routing_table")) {
    doc["gateway_id"] = "master_gateway_1";
    doc["timestamp"] = millis();
    doc["wifi_rssi"] = WiFi.RSSI();
    String jsonOutput;
    serializeJson(doc, jsonOutput);
    client.publish(routing_topic, jsonOutput.c_str());
  } else if (doc.containsKey("sensor_data")) {
    String jsonOutput;
    serializeJson(doc, jsonOutput);
    client.publish(sensor_topic, jsonOutput.c_str());
  }
}

void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();
  processAndPublishData(readFromAtmega());
  delay(2000);
}