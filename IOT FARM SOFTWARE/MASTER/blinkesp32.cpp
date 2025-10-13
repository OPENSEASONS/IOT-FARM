#include <WiFi.h>
#include <SPI.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi Configuration
const char* ssid = "your-ssid";
const char* password = "your-wifi-password";

// MQTT Configuration
const char* mqtt_server = "your-mqtt-server";
const int mqtt_port = 1883;
const char* mqtt_user = "your-username";
const char* mqtt_pass = "your-password";
const char* routing_topic = "lora_mesh/routing_data";
const char* sensor_topic = "lora_mesh/sensor_data";

WiFiClient espClient;
PubSubClient client(espClient);

// SPI Configuration for communication with ATmega328P
#define SPI_CS   5
#define SPI_MISO 19
#define SPI_MOSI 23
#define SPI_SCK  18

// SPI Protocol
#define SPI_START_CMD 0xA5
#define SPI_ACK_CMD 0x06
#define SPI_NODATA_CMD 0x15

void setupWiFi() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  
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
    Serial.print("Attempting MQTT connection...");
    String clientId = "LoRaMeshGateway-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      client.subscribe("lora_mesh/control");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" retrying in 2 seconds...");
      delay(2000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void setup() {
  setupWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  
  // Initialize SPI as Master
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  
  Serial.println("ESP32 Gateway Ready - Waiting for ATmega data...");
}

String readFromAtmega() {
  String result = "";
  digitalWrite(SPI_CS, LOW);
  delayMicroseconds(10);
  
  // Send start command and check for ACK
  byte response = SPI.transfer(SPI_START_CMD);
  
  if (response == SPI_ACK_CMD) {
    // ATmega has data, read it
    for (int i = 0; i < 512; i++) {
      byte data = SPI.transfer(0x00);
      if (data == 0) break; // Null terminator
      result += (char)data;
      delayMicroseconds(50);
    }
  } else if (response == SPI_NODATA_CMD) {
    // No data available
    result = "";
  }
  
  digitalWrite(SPI_CS, HIGH);
  return result;
}

void processAndPublishData(String rawData) {
  if (rawData.length() == 0) return;
  
  Serial.println("Raw data from ATmega: " + rawData);
  
  // Parse the JSON data from ATmega
  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, rawData);
  
  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Check if it's routing data or sensor data
  if (doc.containsKey("routing_table")) {
    // Add gateway metadata to routing data
    doc["gateway_id"] = "master_gateway_1";
    doc["timestamp"] = millis();
    doc["wifi_rssi"] = WiFi.RSSI();
    
    // Convert back to JSON string
    String jsonOutput;
    serializeJson(doc, jsonOutput);
    
    // Publish to routing topic
    if (client.publish(routing_topic, jsonOutput.c_str())) {
      Serial.println("Routing data published to MQTT");
    } else {
      Serial.println("MQTT routing publish failed");
    }
  } else if (doc.containsKey("sensor_data")) {
    // Publish sensor data directly
    String jsonOutput;
    serializeJson(doc, jsonOutput);
    
    if (client.publish(sensor_topic, jsonOutput.c_str())) {
      Serial.println("Sensor data published to MQTT");
    } else {
      Serial.println("MQTT sensor publish failed");
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
  
  // Read data from ATmega every 2 seconds
  String data = readFromAtmega();
  processAndPublishData(data);
  
  delay(2000);
}