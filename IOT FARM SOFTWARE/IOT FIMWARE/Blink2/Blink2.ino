/********************************************************************
 *  LoRa Mesh Node (Nano + RA-02 + Solenoid) – Transceiver + Control
 *  ---------------------------------------------------------------
 *  • RHMesh node (ID=2) – sends local + nearby data to master (ID=1)
 *  • Controls solenoid via D3 (JSON command: {"solenoid": true/false})
 *  • Bundles: local (voltage, solenoid status) + routing table
 *  • Forwards mesh data automatically
 ********************************************************************/

#include <EEPROM.h>
#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <ArduinoJson.h>

// -------------------------- PINS (Matches Schematic) ----------------
#define LED_POWER  4    // Power LED
#define LED_STATUS 5    // Status LED
#define SOLENOID_PIN 3  // MOSFET Gate for Solenoid
#define RF95_CS     10  // LoRa NSS
#define RF95_RST    9   // LoRa Reset
#define RF95_INT    2   // LoRa DIO0
#define VOLTAGE_PIN A0  // Voltage divider

// -------------------------- MESH SETTINGS ------------------------
#define N_NODES 4       // Total nodes (incl. master)
#define NODE_ID_ADDR 0  // EEPROM address for node ID
#define MASTER_ID 1     // Master node
#define SEND_INTERVAL 10000  // ms – send bundle to master
#define FREQUENCY 915.0 // MHz (change to 433.0 if needed)

// -------------------------- GLOBALS -----------------------
uint8_t nodeId;
uint8_t routes[N_NODES];
int16_t rssi[N_NODES];
RH_RF95 rf95(RF95_CS, RF95_INT);
RHMesh *manager = nullptr;
bool solenoidState = false;  // Current solenoid status

// -------------------------- SETUP -------------------------
void setup() {
  pinMode(LED_POWER, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);
  digitalWrite(LED_POWER, HIGH);
  digitalWrite(SOLENOID_PIN, LOW);  // Solenoid off

  Serial.begin(115200);
  while (!Serial);

  // Load node ID from EEPROM
  nodeId = EEPROM.read(NODE_ID_ADDR);
  if (nodeId == 0xFF) {  // First boot
    nodeId = 2;  // Default node 2
    EEPROM.write(NODE_ID_ADDR, nodeId);
  }
  Serial.print("Node ID: "); Serial.println(nodeId);

  // LoRa init
  pinMode(RF95_RST, OUTPUT);
  digitalWrite(RF95_RST, LOW); delay(10);
  digitalWrite(RF95_RST, HIGH); delay(10);

  manager = new RHMesh(rf95, nodeId);
  if (!manager->init()) {
    Serial.println("LoRa init failed");
    while (1);
  }
  rf95.setTxPower(23, false);
  rf95.setFrequency(FREQUENCY);
  rf95.setCADTimeout(500);

  for (uint8_t n = 0; n < N_NODES; n++) {
    routes[n] = 0;
    rssi[n] = 0;
  }

  Serial.println("LoRa Node Ready – Solenoid OFF");
}

// -------------------------- LOOP -------------------------
unsigned long lastSend = 0;

void loop() {
  // Send bundle to master
  if (millis() - lastSend >= SEND_INTERVAL) {
    sendDataBundle();
    lastSend = millis();
  }

  // Receive & process messages (commands or data)
  processIncomingMessage();

  // Blink status LED
  digitalWrite(LED_STATUS, (millis() / 500) % 2);
}

// -------------------------- SEND BUNDLE TO MASTER --------------------
void sendDataBundle() {
  updateRoutingTable();
  String bundle = createNodeJSON();

  uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
  strncpy((char*)buf, bundle.c_str(), sizeof(buf));
  uint8_t len = strlen((char*)buf);

  uint8_t err = manager->sendtoWait(buf, len, MASTER_ID);
  if (err == RH_ROUTER_ERROR_NONE) {
    Serial.println("Sent bundle to master: " + bundle);
    digitalWrite(LED_STATUS, HIGH); delay(100); digitalWrite(LED_STATUS, LOW);
  } else {
    Serial.println("Send failed: " + String(err));
  }
}

// -------------------------- CREATE JSON BUNDLE --------------------
String createNodeJSON() {
  DynamicJsonDocument doc(1024);
  doc["node"] = nodeId;
  doc["solenoid"] = solenoidState ? "ON" : "OFF";
  doc["voltage"] = readVoltage();

  // Add routing table (nearby nodes data)
  JsonArray routingArray = doc.createNestedArray("routing_table");
  for (uint8_t n = 0; n < N_NODES; n++) {
    JsonObject routeObj = routingArray.createNestedObject();
    routeObj["n"] = routes[n];  // Next hop
    routeObj["r"] = rssi[n];    // RSSI
    routeObj["id"] = n + 1;     // Node ID
  }

  // Add any forwarded sensor data from nearby (if received)
  // (In processIncoming, we can store and append here)

  String output;
  serializeJson(doc, output);
  return output;
}

// -------------------------- PROCESS INCOMING --------------------
void processIncomingMessage() {
  uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from, to;

  if (manager->recvfromAckTimeout(buf, &len, 1000, &from, &to)) {
    buf[len] = '\0';
    String msg = (char*)buf;

    Serial.print("From node "); Serial.print(from); Serial.print(": "); Serial.println(msg);

    // Parse JSON command
    DynamicJsonDocument doc(512);
    DeserializationError err = deserializeJson(doc, msg);
    if (!err) {
      if (doc.containsKey("solenoid")) {
        solenoidState = doc["solenoid"].as<bool>();
        digitalWrite(SOLENOID_PIN, solenoidState ? HIGH : LOW);
        Serial.println("Solenoid " + String(solenoidState ? "ON" : "OFF"));
      }
      // If it's sensor data from nearby, store for forwarding (append to bundle next send)
    }

    // Update RSSI for routing
    RHRouter::RoutingTableEntry *route = manager->getRouteTo(from);
    if (route && route->next_hop) {
      rssi[route->next_hop - 1] = rf95.lastRssi();
    }
  }
}

// -------------------------- ROUTING & SENSORS --------------------
void updateRoutingTable() {
  for (uint8_t n = 0; n < N_NODES; n++) {
    RHRouter::RoutingTableEntry *route = manager->getRouteTo(n + 1);
    routes[n] = (n + 1 == nodeId) ? 255 : route->next_hop;
  }
}

float readVoltage() {
  int adc = analogRead(VOLTAGE_PIN);
  float voltage = adc * (12.0 / 1023.0);  // Adjust for divider (e.g., 10k+10k = full scale 12V)
  return voltage;
}