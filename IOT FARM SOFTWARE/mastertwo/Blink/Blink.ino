#include <EEPROM.h>
#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <ArduinoJson.h>

// Pin Definitions (matching schematic for Uno WiFi Rev3)
#define LED_POWER 4    // Second LED on Pin 5
#define LED 3          // Status LED on Pin 9
#define N_NODES 4
#define RF95_CS 10     // LoRa CS on Pin 10
#define RF95_RST 9     // LoRa RESET on Pin 9 (shared with LED)
#define RF95_INT 2     // LoRa DIO0 on Pin 2
#define VOLTAGE_PIN A0 // Voltage divider on A0

// State Machine Definitions
#define STATE_POWER_ON 0
#define STATE_SEND_ROUTING 1
#define STATE_LISTEN_MESSAGES 2
#define STATE_UPDATE_BUNDLE 3
#define STATE_SHUTTING_DOWN 4

// Timing Constants (in milliseconds)
#define SEND_INTERVAL 500
#define LISTEN_INTERVAL 1000
#define BUNDLE_UPDATE_INTERVAL 2000
#define POWER_CHECK_INTERVAL 1000

// Voltage Thresholds (in ADC units, 0-1023 for 0-5V)
#define MIN_VOLTAGE 920  // ~4.5V (1023 * 4.5 / 5)

// Global Variables
uint8_t nodeId;
uint8_t routes[N_NODES];
int16_t rssi[N_NODES];
RH_RF95 rf95(RF95_CS, RF95_INT);
RHMesh *manager = nullptr;
char buf[RH_MESH_MAX_MESSAGE_LEN];
volatile bool dataRequested = false;
volatile uint8_t spiDataIndex = 0;
String meshDataBundle = "";
unsigned long lastSendTime = 0;
unsigned long lastListenTime = 0;
unsigned long lastBundleTime = 0;
unsigned long lastPowerCheckTime = 0;
uint8_t currentState = STATE_POWER_ON;
uint8_t currentNode = 2; // Start with node 2 for routing

// SPI Protocol
#define SPI_START_CMD 0xA5
#define SPI_ACK_CMD 0x06
#define SPI_NODATA_CMD 0x15

// Function Prototypes
void initializeHardware();
int readVoltage();
void updateRoutingTable();
String createRoutingJSON();
void updateMeshDataBundle();
bool sendRoutingInfo(uint8_t targetNode);
void processIncomingMessage();
void handleSPIInterrupt();
void transitionState();
void setLEDs(uint8_t powerState, uint8_t statusState);

void setup() {
  initializeHardware();

  // Initial power check
  if (readVoltage() < MIN_VOLTAGE) {
    currentState = STATE_SHUTTING_DOWN;
  } else {
    Serial.println(F("ATmega328P Master Node Ready"));
    Serial.print(F("Node ID: "));
    Serial.println(nodeId);
  }

  setLEDs(HIGH, LOW); // Power on state
}

void initializeHardware() {
  pinMode(LED_POWER, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(RF95_RST, OUTPUT); // Shared with LED, handle reset carefully
  digitalWrite(RF95_RST, LOW);
  delay(10);
  digitalWrite(RF95_RST, HIGH); // Pulse reset, LED will blink briefly

  Serial.begin(115200);
  while (!Serial);

  nodeId = EEPROM.read(0);
  if (nodeId != 1) {
    nodeId = 1;
    EEPROM.write(0, nodeId);
  }

  manager = new RHMesh(rf95, nodeId);
  if (!manager->init()) {
    Serial.println(F("LoRa init failed"));
    while (1);
  }
  
  rf95.setTxPower(23, false);
  rf95.setFrequency(915.0);  // Check local regulations
  rf95.setCADTimeout(500);

  pinMode(MISO, OUTPUT);  // Uno WiFi Rev3 MISO is Pin 12
  SPCR |= _BV(SPE);
  SPCR |= _BV(SPIE);
  SPI.attachInterrupt();

  for (uint8_t n = 0; n < N_NODES; n++) {
    routes[n] = 0;
    rssi[n] = 0;
  }
}

int readVoltage() {
  int sensorValue = analogRead(VOLTAGE_PIN);
  // Convert ADC value (0-1023) to voltage (0-5V), adjust for divider ratio (12V to 5V)
  float voltage = sensorValue * (5.0 / 1023.0) * (12.0 / 5.0); // Scaled for 12V input
  Serial.print(F("Voltage: "));
  Serial.print(voltage);
  Serial.println(F("V"));
  return sensorValue; // Return ADC value for threshold comparison
}

ISR(SPI_STC_vect) {
  handleSPIInterrupt();
}

void handleSPIInterrupt() {
  byte received = SPDR;
  
  if (received == SPI_START_CMD) {
    if (meshDataBundle.length() > 0) {
      SPDR = SPI_ACK_CMD;
      dataRequested = true;
      spiDataIndex = 0;
    } else {
      SPDR = SPI_NODATA_CMD;
      dataRequested = false;
    }
  } else if (dataRequested) {
    if (spiDataIndex < meshDataBundle.length()) {
      SPDR = meshDataBundle[spiDataIndex++];
    } else {
      SPDR = 0;
      dataRequested = false;
      spiDataIndex = 0;
    }
  }
}

void updateRoutingTable() {
  for (uint8_t n = 0; n < N_NODES; n++) {
    RHRouter::RoutingTableEntry *route = manager->getRouteTo(n + 1);
    if (n + 1 == nodeId) {
      routes[n] = 255; // Self
    } else {
      routes[n] = route->next_hop;
      if (routes[n] == 0) {
        rssi[n] = 0;
      }
    }
  }
}

String createRoutingJSON() {
  DynamicJsonDocument doc(512);
  JsonArray routingArray = doc.to<JsonArray>();
  
  for (uint8_t n = 0; n < N_NODES; n++) {
    JsonObject routeObj = routingArray.createNestedObject();
    routeObj["n"] = routes[n];
    routeObj["r"] = rssi[n];
  }
  
  String output;
  serializeJson(doc, output);
  return output;
}

void updateMeshDataBundle() {
  DynamicJsonDocument doc(1024);
  doc["node"] = nodeId;
  doc["routing_table"] = createRoutingJSON();
  doc["timestamp"] = millis();
  doc["voltage"] = readVoltage() * (5.0 / 1023.0) * (12.0 / 5.0); // Include voltage in JSON
  
  serializeJson(doc, meshDataBundle);
}

bool sendRoutingInfo(uint8_t targetNode) {
  if (targetNode == nodeId) return false;
  
  updateRoutingTable();
  String routeInfo = createRoutingJSON();
  
  uint8_t error = manager->sendtoWait((uint8_t *)routeInfo.c_str(), routeInfo.length(), targetNode);
  if (error == RH_ROUTER_ERROR_NONE) {
    Serial.print(F("Sent routing to node "));
    Serial.println(targetNode);
    RHRouter::RoutingTableEntry *route = manager->getRouteTo(targetNode);
    if (route->next_hop != 0) {
      rssi[route->next_hop - 1] = rf95.lastRssi();
    }
    return true;
  }
  return false;
}

void processIncomingMessage() {
  uint8_t from, len = sizeof(buf);
  uint8_t to;
  
  if (manager->recvfromAckTimeout((uint8_t *)buf, &len, 1000, &from, &to)) {
    buf[len] = '\0';
    
    if (buf[0] == '[') {
      Serial.print(F("Routing from node "));
      Serial.print(from);
      Serial.print(F(": "));
      Serial.println(buf);
    } else if (buf[0] == '{') {
      Serial.print(F("Sensor data from node "));
      Serial.print(from);
      Serial.print(F(": "));
      Serial.println(buf);
      meshDataBundle = buf; // Update bundle with sensor data for ESP32
    }
    
    RHRouter::RoutingTableEntry *route = manager->getRouteTo(from);
    if (route->next_hop != 0) {
      rssi[route->next_hop - 1] = rf95.lastRssi();
    }
  }
}

void setLEDs(uint8_t powerState, uint8_t statusState) {
  digitalWrite(LED_POWER, powerState);
  digitalWrite(LED, statusState);
}

void transitionState() {
  unsigned long currentTime = millis();
  int voltage = readVoltage();

  switch (currentState) {
    case STATE_POWER_ON:
      setLEDs(HIGH, LOW); // Power LED on, Status LED off
      if (currentTime - lastPowerCheckTime >= POWER_CHECK_INTERVAL) {
        if (voltage < MIN_VOLTAGE) {
          currentState = STATE_SHUTTING_DOWN;
        } else {
          currentState = STATE_SEND_ROUTING;
          lastSendTime = currentTime;
        }
        lastPowerCheckTime = currentTime;
      }
      break;

    case STATE_SEND_ROUTING:
      setLEDs(HIGH, HIGH); // Both LEDs on during transmission
      if (currentTime - lastSendTime >= SEND_INTERVAL) {
        if (sendRoutingInfo(currentNode)) {
          currentNode++;
          if (currentNode > N_NODES) currentNode = 2;
        }
        if (currentNode == 2) {
          currentState = STATE_LISTEN_MESSAGES;
          lastListenTime = currentTime;
        }
        lastSendTime = currentTime;
      }
      break;

    case STATE_LISTEN_MESSAGES:
      setLEDs(HIGH, LOW); // Power on, Status off during listen
      if (currentTime - lastListenTime >= LISTEN_INTERVAL) {
        processIncomingMessage();
        currentState = STATE_UPDATE_BUNDLE;
        lastBundleTime = currentTime;
      }
      break;

    case STATE_UPDATE_BUNDLE:
      setLEDs(HIGH, HIGH); // Both on during bundle update
      if (currentTime - lastBundleTime >= BUNDLE_UPDATE_INTERVAL && !dataRequested) {
        updateMeshDataBundle(); // Prepare JSON bundle for ESP32
        currentState = STATE_POWER_ON; // Return to power check cycle
      }
      break;

    case STATE_SHUTTING_DOWN:
      setLEDs(LOW, LOW); // Both LEDs off for power off
      Serial.println(F("Shutting down due to low voltage"));
      while (1) {} // Halt execution
      break;
  }
}

void loop() {
  transitionState();
}