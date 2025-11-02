/********************************************************************
 *  LoRa Mesh Hygrometer Node (Nano + RA-02)
 *  ---------------------------------------------------------------
 *  • RHMesh node (ID stored in EEPROM – default 3)
 *  • Reads capacitive soil-moisture sensor on A0
 *  • Sends local data + routing table to master (node 1)
 *  • NO solenoid control
 ********************************************************************/

#include <EEPROM.h>
#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <ArduinoJson.h>

// -------------------------- PINOUT ------------------------
#define LED_POWER   4      // Power LED
#define LED_STATUS  5      // Status LED
#define RF95_CS    10      // LoRa NSS
#define RF95_RST    9      // LoRa Reset
#define RF95_INT    2      // LoRa DIO0
#define HYGRO_PIN   A0     // Hygrometer analog output
#define VOLTAGE_PIN A1     // Battery voltage divider (optional)

// -------------------------- MESH SETTINGS -----------------
#define N_NODES          4
#define NODE_ID_ADDR     0   // EEPROM address for node ID
#define MASTER_ID        1
#define SEND_INTERVAL 10000  // ms – send bundle to master
#define FREQUENCY      915.0 // MHz

// -------------------------- GLOBALS -----------------------
uint8_t nodeId;
uint8_t routes[N_NODES];
int16_t rssi[N_NODES];

RH_RF95 rf95(RF95_CS, RF95_INT);
RHMesh *manager = nullptr;

// -------------------------- SETUP -------------------------
void setup() {
  pinMode(LED_POWER,  OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_POWER, HIGH);

  Serial.begin(115200);
  while (!Serial);

  // ---- Load node ID from EEPROM ----
  nodeId = EEPROM.read(NODE_ID_ADDR);
  if (nodeId == 0xFF) {               // first boot
    nodeId = 3;                       // default = node 3
    EEPROM.write(NODE_ID_ADDR, nodeId);
  }
  Serial.print(F("Hygrometer Node ID: ")); Serial.println(nodeId);

  // ---- LoRa init ----
  pinMode(RF95_RST, OUTPUT);
  digitalWrite(RF95_RST, LOW);  delay(10);
  digitalWrite(RF95_RST, HIGH); delay(10);

  manager = new RHMesh(rf95, nodeId);
  if (!manager->init()) {
    Serial.println(F("LoRa init failed"));
    while (1);
  }
  rf95.setTxPower(23, false);
  rf95.setFrequency(FREQUENCY);
  rf95.setCADTimeout(500);

  Serial.println(F("Hygrometer Node ready"));
}

// -------------------------- LOOP -------------------------
unsigned long lastSend = 0;

void loop() {
  // ---- Periodic bundle to master ----
  if (millis() - lastSend >= SEND_INTERVAL) {
    sendDataBundle();
    lastSend = millis();
  }

  // ---- Receive any mesh messages (routing updates, etc.) ----
  processIncoming();

  // ---- Status LED blink ----
  digitalWrite(LED_STATUS, (millis() / 500) % 2);
}

// -------------------------- SEND BUNDLE --------------------
// -------------------------- SEND BUNDLE --------------------
// -------------------------- SEND BUNDLE --------------------
void sendDataBundle() {
  updateRoutingTable();
  String bundle = createJSON();

  uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
  strncpy((char*)buf, bundle.c_str(), sizeof(buf));
  uint8_t len = strlen((char*)buf);

  uint8_t err = manager->sendtoWait(buf, len, MASTER_ID);
  if (err == RH_ROUTER_ERROR_NONE) {
    Serial.print(F("Sent to master: "));
    Serial.println(bundle);
    digitalWrite(LED_STATUS, HIGH); delay(100); digitalWrite(LED_STATUS, LOW);
  } else {
    Serial.print(F("Send failed, err="));
    Serial.println(err);
  }
}

// -------------------------- JSON CREATION -----------------
String createJSON() {
  DynamicJsonDocument doc(1024);

  doc["node"]      = nodeId;
  doc["humidity"]  = readHygrometer();   // 0-100 %
  doc["voltage"]   = readBattery();      // V

  // ---- Routing table (same format as master) ----
  JsonArray routing = doc.createNestedArray("routing_table");
  for (uint8_t i = 0; i < N_NODES; ++i) {
    JsonObject o = routing.createNestedObject();
    o["n"] = routes[i];          // next hop
    o["r"] = rssi[i];            // RSSI
    o["id"] = i + 1;             // node number
  }

  String out;
  serializeJson(doc, out);
  return out;
}

// -------------------------- SENSORS -----------------------
float readHygrometer() {
  // Typical capacitive sensor: 0 V = wet, 3.3 V = dry
  int raw = analogRead(HYGRO_PIN);
  // Map to 0-100 % (adjust min/max to your sensor)
  const int AIR_VAL  = 620;   // value in open air
  const int WATER_VAL = 310; // value in water
  int pct = constrain(map(raw, WATER_VAL, AIR_VAL, 100, 0), 0, 100);
  return pct;
}

float readBattery() {
  int raw = analogRead(VOLTAGE_PIN);
  // Divider: 10 kΩ + 10 kΩ → 12 V → 6 V → 5 V ADC scale
  float v = raw * (5.0 / 1023.0) * (12.0 / 5.0);
  return v;
}

// -------------------------- MESH ROUTING -----------------
void updateRoutingTable() {
  for (uint8_t i = 0; i < N_NODES; ++i) {
    uint8_t dest = i + 1;
    RHRouter::RoutingTableEntry *e = manager->getRouteTo(dest);
    routes[i] = (dest == nodeId) ? 255 : e->next_hop;
    if (e->next_hop == 0) rssi[i] = 0;
  }
}

// -------------------------- RECEIVE MESSAGES -------------
void processIncoming() {
  uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from, to;

  if (manager->recvfromAckTimeout(buf, &len, 100, &from, &to)) {
    buf[len] = '\0';
    String msg = (char*)buf;
    Serial.print(F("RX from ")); Serial.print(from); Serial.print(": "); Serial.println(msg);

    // Update RSSI for routing table
    RHRouter::RoutingTableEntry *e = manager->getRouteTo(from);
    if (e && e->next_hop) {
      rssi[e->next_hop - 1] = rf95.lastRssi();
    }
  }
}