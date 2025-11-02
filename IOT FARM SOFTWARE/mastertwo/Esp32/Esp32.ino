/********************************************************************
 *  ESP8266 GATEWAY – SPI Slave (POLLING) + MQTT + Web + p5.js
 *  WORKS ON: NodeMCU, Wemos D1 Mini, Generic ESP8266
 ********************************************************************/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <ESP8266WebServer.h>

// ----------------------- CONFIG -----------------------
const char* ssid = "wutang";
const char* password = "andridge ";
const char* mqtt_server = "YOUR_MQTT_BROKER";
const int   mqtt_port = 1883;
const char* mqtt_user = "";
const char* mqtt_pass = "";


const char* routing_topic = "lora_mesh/routing_data";
const char* sensor_topic  = "lora_mesh/sensor_data";

// ----------------------- SPI -----------------------
#define SPI_START_CMD   0xA5
#define SPI_ACK_CMD     0x06
#define SPI_NODATA_CMD  0x15
#define SPI_CONTROL_CMD 0xC1

String spiBuffer = "";
bool dataReady = false;
bool controlRequested = false;
String controlToSend = "";

// ----------------------- GLOBALS --------------------
WiFiClient espClient;
PubSubClient mqtt(espClient);
ESP8266WebServer server(80);

String latestRouting = "{}";
String latestSensor  = "{}";


void handleRoot();
void handleUpdate();  
void handleData();
void sendControlToUNO(String json);
void reconnectMQTT();


// ----------------------- SETUP ----------------------
void setup() {
  Serial.begin(115200);
  pinMode(SS, INPUT_PULLUP);  // SS pin

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK, IP: " + WiFi.localIP().toString());

  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setBufferSize(1024);

  // SPI Slave
  SPI.begin();
  pinMode(MISO, OUTPUT);
  digitalWrite(MISO, LOW);

  // Web server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/update", HTTP_POST, handleUpdate);
  server.on("/data", HTTP_GET, []() {
    DynamicJsonDocument doc(2048);
    doc["routing"] = latestRouting;
    doc["sensor"]  = latestSensor;
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
  });
  server.begin();
  Serial.println("HTTP server started");
}

// ----------------------- LOOP -----------------------
void loop() {
  if (!mqtt.connected()) reconnectMQTT();
  mqtt.loop();
  server.handleClient();

  pollSPI();

  if (dataReady) {
    processSPIBundle();
    dataReady = false;
  }

  if (controlRequested && digitalRead(SS) == LOW) {
    sendControlToUNO();
    controlRequested = false;
  }
}

// ----------------------- SPI POLLING -----------------------
void pollSPI() {
  static bool inTransfer = false;
  static String buffer = "";

  if (digitalRead(SS) == LOW) {
    if (!inTransfer) {
      uint8_t cmd = SPI.transfer(0);
      if (cmd == SPI_START_CMD) {
        uint8_t ack = SPI.transfer(0);
        if (ack == SPI_ACK_CMD) {
          inTransfer = true;
          buffer = "";
        }
      } else if (cmd == SPI_CONTROL_CMD) {
        SPI.transfer(SPI_ACK_CMD);
        controlRequested = true;
      }
    } else {
      uint8_t c = SPI.transfer(0);
      if (c != 0) {
        buffer += (char)c;
      } else {
        spiBuffer = buffer;
        dataReady = true;
        inTransfer = false;
      }
    }
  } else {
    inTransfer = false;
  }
}

// ----------------------- PROCESS DATA -----------------------
void processSPIBundle() {
  if (spiBuffer.length() == 0) return;

  Serial.println("FROM UNO: " + spiBuffer);

  DynamicJsonDocument doc(1536);
  DeserializationError err = deserializeJson(doc, spiBuffer);
  if (err) {
    Serial.println("JSON ERR: " + String(err.c_str()));
    return;
  }

  doc["gateway_id"] = "master_gateway_1";
  doc["timestamp"] = millis();
  doc["wifi_rssi"] = WiFi.RSSI();

  String out;
  serializeJson(doc, out);

  if (doc.containsKey("routing_table")) {
    latestRouting = out;
    mqtt.publish(routing_topic, out.c_str());
  } else if (doc.containsKey("sensor_data")) {
    latestSensor = out;
    mqtt.publish(sensor_topic, out.c_str());
  }
}

// ----------------------- SEND CONTROL -----------------------
void sendControlToUNO() {
  if (controlToSend.length() == 0) return;

  for (size_t i = 0; i < controlToSend.length(); i++) {
    SPI.transfer(controlToSend[i]);
  }
  SPI.transfer(0);
  Serial.println("SENT TO UNO: " + controlToSend);
  controlToSend = "";
}

// ----------------------- MQTT ------------------------
void reconnectMQTT() {
  while (!mqtt.connected()) {
    String cid = "ESP-GW-" + String(ESP.getChipId(), HEX);
    if (mqtt.connect(cid.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("MQTT OK");
    } else {
      delay(2000);
    }
  }
}

// ----------------------- WEB UI ----------------------
void handleRoot() {
  String html = R"=====(
<!DOCTYPE html><html><head><meta charset="utf-8">
<title>LoRa Mesh</title>
<style>body{font-family:Arial;margin:20px;background:#f5f5f5;}
.container{max-width:1200px;margin:auto;background:white;padding:20px;border-radius:8px;}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:20px;}
.card{border:1px solid #ddd;padding:15px;border-radius:5px;background:#fafafa;}
input{margin:5px;padding:8px;width:200px;}
button{padding:10px;background:#4CAF50;color:white;border:none;border-radius:4px;}
</style>
<script src="https://cdnjs.cloudflare.com/ajax/libs/p5.js/1.7.0/p5.min.js"></script>
</head><body>
<div class="container">
  <h1>LoRa Mesh Gateway</h1>
  <div class="grid">
    <div class="card">
      <h2>Network</h2>
      <div id="viz" style="height:300px;"></div>
    </div>
    <div class="card">
      <h2>Sensor Data</h2>
      <pre id="sensor" style="height:280px;overflow:auto;background:#eee;padding:10px;"></pre>
    </div>
  </div>
  <div class="card" style="margin-top:20px;">
    <h2>UNO Settings</h2>
    <form action="/update" method="post">
      Send Interval: <input name="send" type="number" value="500"><br>
      Listen Interval: <input name="listen" type="number" value="1000"><br>
      Bundle Update: <input name="bundle" type="number" value="2000"><br>
      Min Voltage: <input name="minv" type="number" value="920"><br>
      <button>Apply</button>
    </form>
  </div>
</div>

<script>
let nodes = [], links = [];
function setup() {
  createCanvas(500, 280).parent('viz');
}
function draw() {
  background(245);
  stroke(100); strokeWeight(2);
  links.forEach(l => line(l.x1, l.y1, l.x2, l.y2));
  nodes.forEach(n => {
    fill(n.id===1?'#4CAF50':'#2196F3');
    ellipse(n.x, n.y, 30, 30);
    fill(255); text(n.id, n.x, n.y);
  });
}
setInterval(fetchData, 3000); fetchData();
function fetchData() {
  fetch('/data').then(r=>r.json()).then(d=>{
    nodes=[]; links=[];
    try {
      let r = JSON.parse(d.routing);
      let pos = {}, cx=250, cy=140, rad=100;
      let angle=0, step=TWO_PI/r.routing_table.length;
      r.routing_table.forEach((rt,i)=>{
        let id=i+1;
        let x=cx+rad*Math.cos(angle);
        let y=cy+rad*Math.sin(angle);
        pos[id]={x,y}; nodes.push({id,x,y});
        angle+=step;
      });
      r.routing_table.forEach((rt,i)=>{
        let src=i+1, next=rt.next_hop;
        if(next && next!==255 && next!==0 && pos[next]){
          links.push({x1:pos[src].x, y1:pos[src].y, x2:pos[next].x, y2:pos[next].y});
        }
      });
      document.getElementById('sensor').textContent = JSON.stringify(JSON.parse(d.sensor),null,2);
    } catch(e) {}
  });
}
</script>
</body></html>
)=====";
  server.send(200, "text/html", html);
}

void handleUpdate() {
  String json = "{";
  json += "\"sendInterval\":" + server.arg("send") + ",";
  json += "\"listenInterval\":" + server.arg("listen") + ",";
  json += "\"bundleUpdateInt\":" + server.arg("bundle") + ",";
  json += "\"minVoltageADC\":" + server.arg("minv") + "}";
  
  controlToSend = json;
  server.send(200, "text/html", "<h2>Settings sent to UNO</h2><a href='/'>Back</a>");
}