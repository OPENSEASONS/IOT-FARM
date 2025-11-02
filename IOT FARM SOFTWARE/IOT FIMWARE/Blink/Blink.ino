/********************************************************************
 *  UNO MESH MASTER – LoRa + SPI → ESP8266
 *  ---------------------------------------------------------------
 *  • RadioHead RHMesh master (node 1)
 *  • Builds routing table + sensor bundle
 *  • Sends JSON to ESP8266 via SPI (master)
 *  • Receives control JSON from ESP8266 (slave → master)
 *  • Adjustable constants stored in EEPROM
 ********************************************************************/

#include <EEPROM.h>
#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <ArduinoJson.h>

// -------------------------- PINOUT --------------------------
#define LED_POWER   4
#define LED_STATUS  3
#define RF95_CS    10
#define RF95_RST    9
#define RF95_INT    2
#define VOLTAGE_PIN A0

// -------------------------- SETTINGS ------------------------
#define N_NODES 4
#define SEND_INTERVAL       500   // ms – routing broadcast
#define LISTEN_INTERVAL    1000   // ms – receive window
#define BUNDLE_UPDATE_INT  2000   // ms – JSON bundle rebuild
#define POWER_CHECK_INT    1000   // ms – voltage check
#define MIN_VOLTAGE_ADC     920   // ~4.5 V on 5 V scale

// -------------------------- SPI ---------------------------
#define SPI_START_CMD   0xA5
#define SPI_ACK_CMD     0x06
#define SPI_NODATA_CMD  0x15
#define SPI_CONTROL_CMD 0xC1   // ESP → UNO control

// -------------------------- GLOBALS -----------------------
uint8_t nodeId = 1;
uint8_t routes[N_NODES] = {0};
int16_t rssi[N_NODES]   = {0};

RH_RF95 rf95(RF95_CS, RF95_INT);
RHMesh *manager = nullptr;

String meshBundle = "";          // JSON sent to ESP
String controlIn  = "";          // JSON received from ESP
volatile bool spiTxPending = false;
volatile uint16_t spiTxIdx = 0;

// Adjustable constants (stored in EEPROM)
struct Settings {
  uint16_t sendInterval;
  uint16_t listenInterval;
  uint16_t bundleUpdateInt;
  uint16_t minVoltageADC;
};
Settings cfg;

// -------------------------- SETUP -------------------------
void setup() {
  pinMode(LED_POWER, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  digitalWrite(LED_POWER, HIGH);

  Serial.begin(115200);
  while (!Serial);

  loadSettings();

  // ---- LoRa init ----
  pinMode(RF95_RST, OUTPUT);
  digitalWrite(RF95_RST, LOW); delay(10);
  digitalWrite(RF95_RST, HIGH); delay(10);

  manager = new RHMesh(rf95, nodeId);
  if (!manager->init()) { Serial.println(F("LoRa init fail")); while (1); }
  rf95.setTxPower(23, false);
  rf95.setFrequency(915.0);
  rf95.setCADTimeout(500);

  // ---- SPI Master ----
  SPI.begin();
  pinMode(MISO, INPUT);   // not used as master
  SPI.setClockDivider(SPI_CLOCK_DIV16); // ~1 MHz

  Serial.println(F("UNO Mesh Master ready"));
}

// -------------------------- LOOP -------------------------
unsigned long tSend = 0, tListen = 0, tBundle = 0, tPower = 0;
uint8_t state = 0; // 0: power check, 1: send routing, 2: listen, 3: bundle
uint8_t curNode = 2;

void loop() {
  unsigned long now = millis();

  // ---- STATE MACHINE ----
  switch (state) {
    case 0: // Power check
      if (now - tPower >= POWER_CHECK_INT) {
        if (readVoltageADC() < cfg.minVoltageADC) {
          shutdownLowVoltage();
        }
        state = 1;
        tSend = now;
      }
      break;

    case 1: // Send routing to next node
      digitalWrite(LED_STATUS, HIGH);
      if (now - tSend >= cfg.sendInterval) {
        sendRoutingInfo(curNode);
        curNode = (curNode >= N_NODES) ? 2 : curNode + 1;
        if (curNode == 2) { state = 2; tListen = now; }
        tSend = now;
      }
      break;

    case 2: // Listen for incoming messages
      digitalWrite(LED_STATUS, LOW);
      if (now - tListen >= cfg.listenInterval) {
        processIncoming();
        state = 3;
        tBundle = now;
      }
      break;

    case 3: // Build bundle & send to ESP via SPI
      digitalWrite(LED_STATUS, HIGH);
      if (now - tBundle >= cfg.bundleUpdateInt) {
        buildBundle();
        sendBundleToESP();
        state = 0;
        tPower = now;
      }
      break;
  }

  // ---- Handle incoming control from ESP (polled) ----
  receiveControlFromESP();

  // ---- Process any control commands ----
  if (controlIn.length()) {
    applyControl(controlIn);
    controlIn = "";
  }
}

// -------------------------- HARDWARE -------------------------
int readVoltageADC() {
  int adc = analogRead(VOLTAGE_PIN);
  float v = adc * (5.0 / 1023.0) * (12.0 / 5.0);
  Serial.print(F("Batt: ")); Serial.print(v, 2); Serial.println(F("V"));
  return adc;
}

void shutdownLowVoltage() {
  Serial.println(F("LOW VOLTAGE – SHUTDOWN"));
  digitalWrite(LED_POWER, LOW);
  digitalWrite(LED_STATUS, LOW);
  while (1);
}

// -------------------------- LoRa -------------------------
void updateRoutingTable() {
  for (uint8_t i = 0; i < N_NODES; ++i) {
    uint8_t dest = i + 1;
    RHRouter::RoutingTableEntry *e = manager->getRouteTo(dest);
    routes[i] = (dest == nodeId) ? 255 : e->next_hop;
    if (e->next_hop == 0) rssi[i] = 0;
  }
}

String routingJSON() {
  DynamicJsonDocument doc(512);
  JsonArray arr = doc.to<JsonArray>();
  for (uint8_t i = 0; i < N_NODES; ++i) {
    JsonObject o = arr.createNestedObject();
    o["n"] = routes[i];
    o["r"] = rssi[i];
  }
  String out;
  serializeJson(doc, out);
  return out;
}

bool sendRoutingInfo(uint8_t target) {
  if (target == nodeId) return false;
  updateRoutingTable();
  String payload = routingJSON();

  uint8_t err = manager->sendtoWait((uint8_t*)payload.c_str(),
                                   payload.length(), target);
  if (err == RH_ROUTER_ERROR_NONE) {
    RHRouter::RoutingTableEntry *e = manager->getRouteTo(target);
    if (e->next_hop) rssi[e->next_hop - 1] = rf95.lastRssi();
    return true;
  }
  return false;
}

void processIncoming() {
  uint8_t buf[RH_MESH_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
  uint8_t from, to;

  if (manager->recvfromAckTimeout(buf, &len, 200, &from, &to)) {
    buf[len] = '\0';
    String msg = (char*)buf;

    if (msg.startsWith("{")) {
      // Sensor data → store for bundle
      meshBundle = msg;
      Serial.print(F("Sensor from ")); Serial.println(from);
    } else if (msg.startsWith("[")) {
      Serial.print(F("Routing from ")); Serial.println(from);
    }

    RHRouter::RoutingTableEntry *e = manager->getRouteTo(from);
    if (e->next_hop) rssi[e->next_hop - 1] = rf95.lastRssi();
  }
}

// -------------------------- JSON BUNDLE --------------------
void buildBundle() {
  updateRoutingTable();
  DynamicJsonDocument doc(1024);
  doc["node"] = nodeId;
  doc["routing_table"] = serialized(routingJSON());
  doc["timestamp"] = millis();
  doc["voltage"] = readVoltageADC() * (5.0 / 1023.0) * (12.0 / 5.0);
  if (meshBundle.length()) {
    DeserializationError err = deserializeJson(doc["sensor_data"], meshBundle);
    if (err) doc["sensor_data"] = nullptr;
  }
  meshBundle = "";
  serializeJson(doc, meshBundle);
}

// -------------------------- SPI TO ESP8266 -----------------
void sendBundleToESP() {
  if (meshBundle.length() == 0) return;

  digitalWrite(SS, LOW);
  delayMicroseconds(10);
  SPI.transfer(SPI_START_CMD);
  uint8_t ack = SPI.transfer(0);
  if (ack == SPI_ACK_CMD) {
    for (size_t i = 0; i < meshBundle.length(); ++i)
      SPI.transfer(meshBundle[i]);
    SPI.transfer(0); // terminator
  }
  digitalWrite(SS, HIGH);
}

// ---- Receive control JSON from ESP (ESP initiates) ----
void receiveControlFromESP() {
  digitalWrite(SS, LOW);
  delayMicroseconds(10);
  SPI.transfer(SPI_CONTROL_CMD);
  uint8_t ack = SPI.transfer(0);
  if (ack == SPI_ACK_CMD) {
    controlIn = "";
    while (true) {
      uint8_t c = SPI.transfer(0);
      if (c == 0) break;
      controlIn += (char)c;
    }
  }
  digitalWrite(SS, HIGH);
}

// -------------------------- CONTROL FROM WEB ----------------
void applyControl(String json) {
  DynamicJsonDocument doc(512);
  DeserializationError err = deserializeJson(doc, json);
  if (err) { Serial.println(err.c_str()); return; }

  if (doc.containsKey("sendInterval")) cfg.sendInterval = doc["sendInterval"];
  if (doc.containsKey("listenInterval")) cfg.listenInterval = doc["listenInterval"];
  if (doc.containsKey("bundleUpdateInt")) cfg.bundleUpdateInt = doc["bundleUpdateInt"];
  if (doc.containsKey("minVoltageADC")) cfg.minVoltageADC = doc["minVoltageADC"];

  saveSettings();
  Serial.println(F("Settings updated"));
}

// -------------------------- EEPROM -----------------------
void loadSettings() {
  EEPROM.get(0, cfg);
  if (cfg.sendInterval == 0xFFFF) { // first boot
    cfg.sendInterval = SEND_INTERVAL;
    cfg.listenInterval = LISTEN_INTERVAL;
    cfg.bundleUpdateInt = BUNDLE_UPDATE_INT;
    cfg.minVoltageADC = MIN_VOLTAGE_ADC;
    saveSettings();
  }
}
void saveSettings() { EEPROM.put(0, cfg); }