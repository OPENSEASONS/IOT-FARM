#include <EEPROM.h>
#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <ArduinoJson.h>

#define LED 13
#define N_NODES 4
#define HYGROMETER_PIN A0  // Hygrometer connected to analog pin A0

uint8_t nodeId;
uint8_t routes[N_NODES];
int16_t rssi[N_NODES];

RH_RF95 rf95;
RHMesh *manager;

char buf[RH_MESH_MAX_MESSAGE_LEN];

// Hygrometer calibration
const int dryValue = 0;    // Value when completely dry
const int wetValue = 1023; // Value when completely wet

void setup() {
  pinMode(LED, OUTPUT);
  pinMode(HYGROMETER_PIN, INPUT);
  Serial.begin(115200);
  
  // Read node ID from EEPROM (set this differently for each node: 2, 3, 4)
  nodeId = EEPROM.read(0);
  if (nodeId < 2 || nodeId > N_NODES) {
    nodeId = 2; // Change this for each node (2, 3, or 4)
    EEPROM.write(0, nodeId);
  }

  manager = new RHMesh(rf95, nodeId);
  if (!manager->init()) {
    Serial.println(F("LoRa init failed"));
    while (1);
  }
  
  rf95.setTxPower(23, false);
  rf95.setFrequency(915.0);
  rf95.setCADTimeout(500);

  // Initialize routing table
  for(uint8_t n=1; n<=N_NODES; n++) {
    routes[n-1] = 0;
    rssi[n-1] = 0;
  }

  Serial.print(F("Sensor Node "));
  Serial.print(nodeId);
  Serial.println(F(" Ready"));
}

int readSoilMoisture() {
  int sensorValue = analogRead(HYGROMETER_PIN);
  // Convert to percentage (invert because higher value = drier soil)
  int moisturePercent = map(sensorValue, dryValue, wetValue, 100, 0);
  moisturePercent = constrain(moisturePercent, 0, 100);
  return moisturePercent;
}

String createSensorDataJSON() {
  DynamicJsonDocument doc(256);
  doc["node"] = nodeId;
  doc["sensor_type"] = "hygrometer";
  doc["sensor_data"] = readSoilMoisture();
  doc["raw_value"] = analogRead(HYGROMETER_PIN);
  doc["timestamp"] = millis();
  doc["battery"] = 100; // Placeholder for battery level
  
  String output;
  serializeJson(doc, output);
  return output;
}

void updateRoutingTable() {
  for(uint8_t n=1; n<=N_NODES; n++) {
    RHRouter::RoutingTableEntry *route = manager->getRouteTo(n);
    if (n == nodeId) {
      routes[n-1] = 255; // Self
    } else {
      routes[n-1] = route->next_hop;
      if (routes[n-1] == 0) {
        rssi[n-1] = 0;
      }
    }
  }
}

void sendRoutingInfo(uint8_t targetNode) {
  if (targetNode == nodeId) return;
  
  updateRoutingTable();
  
  // Create routing info string in your specified format
  String routeInfo = "[";
  for(uint8_t n=1; n<=N_NODES; n++) {
    routeInfo += "{\"n\":";
    routeInfo += routes[n-1];
    routeInfo += ",\"r\":";
    routeInfo += rssi[n-1];
    routeInfo += "}";
    if (n < N_NODES) routeInfo += ",";
  }
  routeInfo += "]";
  
  // Send to target node (node 1 - master gateway)
  uint8_t error = manager->sendtoWait((uint8_t *)routeInfo.c_str(), routeInfo.length(), targetNode);
  
  if (error == RH_ROUTER_ERROR_NONE) {
    Serial.print(F("Routing->"));
    Serial.println(targetNode);
    
    // Update RSSI
    RHRouter::RoutingTableEntry *route = manager->getRouteTo(targetNode);
    if (route->next_hop != 0) {
      rssi[route->next_hop-1] = rf95.lastRssi();
    }
  }
}

void sendSensorData() {
  String sensorData = createSensorDataJSON();
  
  // Send sensor data to master gateway (node 1)
  uint8_t error = manager->sendtoWait((uint8_t *)sensorData.c_str(), sensorData.length(), 1);
  
  if (error == RH_ROUTER_ERROR_NONE) {
    Serial.print(F("Sensor data sent: "));
    Serial.println(sensorData);
    
    // Update RSSI for master gateway route
    RHRouter::RoutingTableEntry *route = manager->getRouteTo(1);
    if (route->next_hop != 0) {
      rssi[route->next_hop-1] = rf95.lastRssi();
    }
  } else {
    Serial.println(F("Failed to send sensor data"));
  }
}

void processIncomingMessage() {
  uint8_t from, len = sizeof(buf);
  uint8_t to;
  
  if (manager->recvfromAckTimeout((uint8_t *)buf, &len, 1000, &from, &to)) {
    buf[len] = '\0';
    Serial.print(from);
    Serial.print(F("->: "));
    Serial.println(buf);
    
    // Update RSSI for the route
    RHRouter::RoutingTableEntry *route = manager->getRouteTo(from);
    if (route->next_hop != 0) {
      rssi[route->next_hop-1] = rf95.lastRssi();
    }
  }
}

void loop() {
  digitalWrite(LED, HIGH);
  
  static unsigned long lastSensorSend = 0;
  static unsigned long lastRoutingSend = 0;
  
  // Send sensor data every 30 seconds
  if (millis() - lastSensorSend > 30000) {
    sendSensorData();
    lastSensorSend = millis();
  }
  
  // Send routing info every 10 seconds
  if (millis() - lastRoutingSend > 10000) {
    sendRoutingInfo(1); // Send to master gateway
    lastRoutingSend = millis();
  }
  
  // Listen for messages
  processIncomingMessage();
  
  digitalWrite(LED, LOW);
  delay(1000);
}