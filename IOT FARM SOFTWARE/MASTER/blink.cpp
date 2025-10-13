#include <EEPROM.h>
#include <RHRouter.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <ArduinoJson.h>

#define LED 9
#define N_NODES 4

uint8_t nodeId;
uint8_t routes[N_NODES];
int16_t rssi[N_NODES];

RH_RF95 rf95;
RHMesh *manager;

char buf[RH_MESH_MAX_MESSAGE_LEN];
volatile bool dataRequested = false;
volatile uint8_t spiDataIndex = 0;

String meshDataBundle = "";

// SPI Protocol
#define SPI_START_CMD 0xA5
#define SPI_ACK_CMD 0x06
#define SPI_NODATA_CMD 0x15

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  while (!Serial);

  // Read node ID from EEPROM - Master is node 1
  nodeId = EEPROM.read(0);
  if (nodeId != 1) {
    nodeId = 1;
    EEPROM.write(0, nodeId);
  }

  // Initialize LoRa mesh
  manager = new RHMesh(rf95, nodeId);
  if (!manager->init()) {
    Serial.println(F("LoRa init failed"));
    while (1);
  }
  
  rf95.setTxPower(23, false);
  rf95.setFrequency(915.0);
  rf95.setCADTimeout(500);

  // Configure as SPI slave for ESP32 communication
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  SPCR |= _BV(SPIE);
  SPI.attachInterrupt();

  // Initialize routing table
  for(uint8_t n=1; n<=N_NODES; n++) {
    routes[n-1] = 0;
    rssi[n-1] = 0;
  }

  Serial.println(F("ATmega328P Master Node Ready"));
  Serial.print(F("Node ID: "));
  Serial.println(nodeId);
}

// SPI Interrupt Service Routine
ISR(SPI_STC_vect) {
  byte received = SPDR;
  
  if (received == SPI_START_CMD) {
    // ESP32 is requesting data
    if (meshDataBundle.length() > 0) {
      SPDR = SPI_ACK_CMD;
      dataRequested = true;
      spiDataIndex = 0;
    } else {
      SPDR = SPI_NODATA_CMD;
      dataRequested = false;
    }
  } else if (dataRequested) {
    // Send data to ESP32
    if (spiDataIndex < meshDataBundle.length()) {
      SPDR = meshDataBundle[spiDataIndex++];
    } else {
      SPDR = 0; // End of data
      dataRequested = false;
      spiDataIndex = 0;
    }
  }
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

String createRoutingJSON() {
  DynamicJsonDocument doc(512);
  JsonArray routingArray = doc.to<JsonArray>();
  
  for(uint8_t n=1; n<=N_NODES; n++) {
    JsonObject routeObj = routingArray.createNestedObject();
    routeObj["n"] = routes[n-1];
    routeObj["r"] = rssi[n-1];
  }
  
  String output;
  serializeJson(doc, output);
  return output;
}

void updateMeshDataBundle() {
  DynamicJsonDocument doc(1024);
  doc["node"] = nodeId;
  doc["routing_table"] = serialized(createRoutingJSON());
  doc["timestamp"] = millis();
  
  serializeJson(doc, meshDataBundle);
}

void sendRoutingInfo(uint8_t targetNode) {
  if (targetNode == nodeId) return;
  
  updateRoutingTable();
  String routeInfo = createRoutingJSON();
  
  // Send routing info to target node
  uint8_t error = manager->sendtoWait((uint8_t *)routeInfo.c_str(), routeInfo.length(), targetNode);
  
  if (error == RH_ROUTER_ERROR_NONE) {
    Serial.print(F("Sent routing to node "));
    Serial.println(targetNode);
    
    // Update RSSI for the route
    RHRouter::RoutingTableEntry *route = manager->getRouteTo(targetNode);
    if (route->next_hop != 0) {
      rssi[route->next_hop-1] = rf95.lastRssi();
    }
  }
}

void processIncomingMessage() {
  uint8_t from, len = sizeof(buf);
  uint8_t to;
  
  if (manager->recvfromAckTimeout((uint8_t *)buf, &len, 1000, &from, &to)) {
    buf[len] = '\0';
    
    // Check if it's routing data or sensor data
    if (buf[0] == '[') {
      // It's routing data
      Serial.print(F("Routing from node "));
      Serial.print(from);
      Serial.print(F(": "));
      Serial.println(buf);
    } else if (buf[0] == '{') {
      // It's sensor data - forward to ESP32 via SPI bundle
      Serial.print(F("Sensor data from node "));
      Serial.print(from);
      Serial.print(F(": "));
      Serial.println(buf);
      
      // Store sensor data for ESP32
      meshDataBundle = buf;
    }
    
    // Update RSSI for this route
    RHRouter::RoutingTableEntry *route = manager->getRouteTo(from);
    if (route->next_hop != 0) {
      rssi[route->next_hop-1] = rf95.lastRssi();
    }
  }
}

void loop() {
  digitalWrite(LED, HIGH);
  
  // Send routing info to each node
  for(uint8_t n=2; n<=N_NODES; n++) {
    sendRoutingInfo(n);
    delay(500);
  }
  
  // Listen for incoming messages (both routing and sensor data)
  processIncomingMessage();
  
  // Update routing data bundle for ESP32 (if no sensor data is present)
  if (!dataRequested) {
    updateMeshDataBundle();
  }
  
  digitalWrite(LED, LOW);
  delay(2000);
}