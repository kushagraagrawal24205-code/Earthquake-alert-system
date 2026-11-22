/*
  EARTHQUAKE MONITORING SYSTEM - RECEIVER
  Board: ESP8266 NodeMCU
  LoRa: SX1278 433 MHz
  OLED: SSD1306 128x64 I2C
  Dashboard: ESP8266 local Wi-Fi access point

  Receiver wiring:
    SX1278 NSS   -> D8
    SX1278 DIO0  -> D1
    SX1278 RESET -> D0
    SX1278 SCK   -> D5
    SX1278 MISO  -> D6
    SX1278 MOSI  -> D7
    SX1278 VCC   -> 3.3V
    SX1278 GND   -> GND

    OLED SDA -> D2
    OLED SCL -> D4
    OLED VCC -> 3.3V (as supported by your module)
    OLED GND -> GND

    Buzzer signal -> D3
    Buzzer GND    -> GND

  Required libraries:
    - RadioLib
    - Adafruit GFX Library
    - Adafruit SSD1306

  Expected transmitter packet:
    V1,sequence,status,ax,ay,az,rms,peak,frequency

  Example:
    V1,15,ALERT,0.12,-0.08,9.79,0.34,1.24,3.91

  IMPORTANT:
    Upload the matching upgraded transmitter sketch as well.
    This receiver does not understand the older packet format:
    NORMAL,0.45,#12
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ---------------- PIN CONFIGURATION ----------------
#define LORA_NSS   D8
#define LORA_DIO0  D1
#define LORA_RESET D0

#define OLED_SDA   D2
#define OLED_SCL   D4
#define BUZZER_PIN D3

// ---------------- RADIO / DISPLAY ------------------
SX1278 radio = new Module(LORA_NSS, LORA_DIO0, LORA_RESET, RADIOLIB_NC);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

ESP8266WebServer server(80);

// ---------------- WI-FI ACCESS POINT ----------------
const char* AP_SSID = "EARTHQUAKE_MONITOR";
const char* AP_PASSWORD = "quake1234";

// ---------------- RSSI CALIBRATION ------------------
// User's outdoor line-of-sight measurements.
// Estimated distance is approximate and environment-dependent.
const int CAL_COUNT = 7;
const float calDistance[CAL_COUNT] = {1, 5, 10, 20, 30, 40, 60};
const float calRSSI[CAL_COUNT]      = {-30, -44, -51, -55, -65, -72, -90};

// ---------------- RECEIVED TELEMETRY ----------------
volatile bool receivedFlag = false;

bool dataAvailable = false;
bool alertActive = false;
bool previousAlert = false;

uint32_t packetCount = 0;
uint32_t missedPackets = 0;
uint32_t lastSequence = 0;

float accelX = 0.0f;
float accelY = 0.0f;
float accelZ = 0.0f;
float rmsValue = 0.0f;
float peakValue = 0.0f;
float dominantFrequency = 0.0f;

float lastRSSI = 0.0f;
float estimatedDistance = -1.0f;

unsigned long lastPacketMillis = 0;
unsigned long lastDisplayMillis = 0;
unsigned long lastBuzzerMillis = 0;

const unsigned long PACKET_TIMEOUT_MS = 5000;
const unsigned long DISPLAY_REFRESH_MS = 300;

// ---------------- STARTUP SEQUENCE ------------------
bool startupSequenceActive = true;
unsigned long startupStartTime = 0;
uint8_t startupScreenShown = 255;

// ---------------- DASHBOARD HISTORY -----------------
const uint8_t HISTORY_SIZE = 60;
float histX[HISTORY_SIZE];
float histY[HISTORY_SIZE];
float histZ[HISTORY_SIZE];
float histRMS[HISTORY_SIZE];
float histPeak[HISTORY_SIZE];
float histFreq[HISTORY_SIZE];
unsigned long histTime[HISTORY_SIZE];
uint8_t historyHead = 0;
uint8_t historyCount = 0;

String lastPacketText = "Waiting for transmitter";

// ---------------- INTERRUPT -------------------------
#if defined(ESP8266)
ICACHE_RAM_ATTR
#endif
void setReceivedFlag(void) {
  receivedFlag = true;
}

// ---------------- RSSI TO DISTANCE ------------------
float estimateDistanceFromRSSI(float rssi) {
  if (rssi > calRSSI[0] || rssi < calRSSI[CAL_COUNT - 1]) {
    return -1.0f;
  }

  for (int i = 0; i < CAL_COUNT - 1; i++) {
    if (rssi <= calRSSI[i] && rssi >= calRSSI[i + 1]) {
      float ratio = (rssi - calRSSI[i]) /
                    (calRSSI[i + 1] - calRSSI[i]);
      return calDistance[i] +
             ratio * (calDistance[i + 1] - calDistance[i]);
    }
  }
  return -1.0f;
}

// ---------------- OLED STARTUP SCREENS ---------------
void drawStartupScreen(uint8_t screenNumber) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (screenNumber == 0) {
    display.setTextSize(2);
    display.setCursor(20, 15);
    display.println(F("WELCOME!"));
    display.setTextSize(1);
    display.setCursor(19, 43);
    display.println(F("System Initializing"));
  }
  else if (screenNumber == 1) {
    display.setTextSize(1);
    display.setCursor(10, 5);
    display.println(F("PROJECT"));
    display.setTextSize(2);
    display.setCursor(4, 23);
    display.println(F("EARTHQUAKE"));
    display.setTextSize(1);
    display.setCursor(25, 49);
    display.println(F("MONITORING SYSTEM"));
  }
  else if (screenNumber == 2) {
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println(F("DEVELOPED BY:"));
    display.setCursor(0, 15);
    display.println(F("Kushagra Agrawal 23EC29"));
    display.setCursor(0, 29);
    display.println(F("Rahul Dwivedi 23EC45"));
    display.setCursor(0, 43);
    display.println(F("Kapil Yadav 23EC27"));
  }

  display.display();
}

void updateStartupSequence() {
  if (!startupSequenceActive) return;

  unsigned long elapsed = millis() - startupStartTime;
  uint8_t desiredScreen;

  if (elapsed < 2000) {
    desiredScreen = 0;
  } else if (elapsed < 4000) {
    desiredScreen = 1;
  } else if (elapsed < 7000) {
    desiredScreen = 2;
  } else {
    startupSequenceActive = false;
    display.clearDisplay();
    display.display();
    return;
  }

  if (desiredScreen != startupScreenShown) {
    startupScreenShown = desiredScreen;
    drawStartupScreen(desiredScreen);
  }
}

// ---------------- OLED LIVE DISPLAY -----------------
void drawLiveOLED() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(F("EARTHQUAKE MONITOR"));

  display.setCursor(0, 11);
  display.print(F("X:")); display.print(accelX, 2);
  display.print(F(" Y:")); display.print(accelY, 2);

  display.setCursor(0, 21);
  display.print(F("Z:")); display.print(accelZ, 2);
  display.print(F(" m/s2"));

  display.setCursor(0, 32);
  display.print(F("RMS:")); display.print(rmsValue, 2);
  display.print(F(" Peak:")); display.print(peakValue, 2);

  display.setCursor(0, 43);
  display.print(F("Freq:")); display.print(dominantFrequency, 2);
  display.print(F(" Hz"));

  display.setCursor(0, 54);
  if (!dataAvailable || millis() - lastPacketMillis > PACKET_TIMEOUT_MS) {
    display.print(F("NO DATA"));
  } else if (alertActive) {
    display.print(F("!!! EARTHQUAKE ALERT !!!"));
  } else {
    display.print(F("STATUS: NORMAL"));
  }

  display.display();
}

// ---------------- HISTORY STORAGE -------------------
void addHistoryPoint() {
  histX[historyHead] = accelX;
  histY[historyHead] = accelY;
  histZ[historyHead] = accelZ;
  histRMS[historyHead] = rmsValue;
  histPeak[historyHead] = peakValue;
  histFreq[historyHead] = dominantFrequency;
  histTime[historyHead] = millis();

  historyHead = (historyHead + 1) % HISTORY_SIZE;
  if (historyCount < HISTORY_SIZE) historyCount++;
}

// ---------------- PACKET PARSING ---------------------
bool parseTelemetryPacket(const String& packet) {
  char statusText[8] = {0};
  unsigned long sequence = 0;
  float x, y, z, rms, peak, freq;

  int parsed = sscanf(packet.c_str(),
                      "V1,%lu,%7[^,],%f,%f,%f,%f,%f,%f",
                      &sequence, statusText,
                      &x, &y, &z, &rms, &peak, &freq);

  if (parsed != 8) return false;

  if (strcmp(statusText, "NORMAL") != 0 &&
      strcmp(statusText, "ALERT") != 0) {
    return false;
  }

  // Count missing sequence numbers. Assumes monotonically increasing sequence.
  if (packetCount > 0 && sequence > lastSequence + 1) {
    missedPackets += (sequence - lastSequence - 1);
  }

  lastSequence = sequence;
  packetCount++;

  accelX = x;
  accelY = y;
  accelZ = z;
  rmsValue = rms;
  peakValue = peak;
  dominantFrequency = freq;

  previousAlert = alertActive;
  alertActive = (strcmp(statusText, "ALERT") == 0);

  lastPacketMillis = millis();
  dataAvailable = true;
  lastPacketText = packet;

  lastRSSI = radio.getRSSI();
  estimatedDistance = estimateDistanceFromRSSI(lastRSSI);

  addHistoryPoint();

  // One short beep only on transition from NORMAL to ALERT.
  if (alertActive && !previousAlert) {
    tone(BUZZER_PIN, 2200, 4000);
    lastBuzzerMillis = millis();
  }

  return true;
}

// ---------------- RADIO SERVICE ----------------------
void serviceLoRa() {
  bool flagCopy = false;

  noInterrupts();
  if (receivedFlag) {
    receivedFlag = false;
    flagCopy = true;
  }
  interrupts();

  if (!flagCopy) return;

  String incoming;
  int state = radio.readData(incoming);

  if (state == RADIOLIB_ERR_NONE) {
    incoming.trim();
    if (!parseTelemetryPacket(incoming)) {
      Serial.print(F("Invalid packet: "));
      Serial.println(incoming);
    } else {
      Serial.print(F("RX: "));
      Serial.print(incoming);
      Serial.print(F(" | RSSI: "));
      Serial.println(lastRSSI);
    }
  } else {
    Serial.print(F("LoRa read error: "));
    Serial.println(state);
  }

  radio.startReceive();
}

// ---------------- JSON HELPERS -----------------------
String jsonFloat(float value, uint8_t decimals = 2) {
  if (isnan(value) || isinf(value)) return "null";
  return String(value, decimals);
}

String buildDataJSON() {
  bool connected = dataAvailable &&
                   (millis() - lastPacketMillis <= PACKET_TIMEOUT_MS);

  String json;
  json.reserve(3500);

  json = "{";
  json += "\"connected\":" + String(connected ? "true" : "false") + ",";
  json += "\"status\":\"" + String(connected ? (alertActive ? "ALERT" : "NORMAL") : "NO DATA") + "\",";
  json += "\"ax\":" + jsonFloat(accelX) + ",";
  json += "\"ay\":" + jsonFloat(accelY) + ",";
  json += "\"az\":" + jsonFloat(accelZ) + ",";
  json += "\"rms\":" + jsonFloat(rmsValue) + ",";
  json += "\"peak\":" + jsonFloat(peakValue) + ",";
  json += "\"frequency\":" + jsonFloat(dominantFrequency) + ",";
  json += "\"rssi\":" + jsonFloat(lastRSSI, 1) + ",";
  json += "\"distance\":" + jsonFloat(estimatedDistance, 1) + ",";
  json += "\"packets\":" + String(packetCount) + ",";
  json += "\"missed\":" + String(missedPackets) + ",";
  json += "\"sequence\":" + String(lastSequence) + ",";
  json += "\"age_ms\":" + String(dataAvailable ? millis() - lastPacketMillis : 0) + ",";
  json += "\"history\":[";

  for (uint8_t i = 0; i < historyCount; i++) {
    uint8_t idx = (historyHead + HISTORY_SIZE - historyCount + i) % HISTORY_SIZE;
    if (i) json += ",";
    json += "{";
    json += "\"t\":" + String(histTime[idx]) + ",";
    json += "\"x\":" + jsonFloat(histX[idx]) + ",";
    json += "\"y\":" + jsonFloat(histY[idx]) + ",";
    json += "\"z\":" + jsonFloat(histZ[idx]) + ",";
    json += "\"rms\":" + jsonFloat(histRMS[idx]) + ",";
    json += "\"peak\":" + jsonFloat(histPeak[idx]) + ",";
    json += "\"freq\":" + jsonFloat(histFreq[idx]);
    json += "}";
  }

  json += "]}";
  return json;
}

// ---------------- DASHBOARD HTML --------------------
const char INDEX_HTML[] PROGMEM = R"=====(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Earthquake Monitoring System</title>
<style>
:root{--bg:#07111f;--panel:#101e30;--panel2:#14263b;--text:#e8f1fb;--muted:#91a6bd;--cyan:#46d9f2;--green:#42df9b;--red:#ff5e6c;--line:#263c55}
*{box-sizing:border-box}
body{margin:0;background:linear-gradient(140deg,#07111f,#0b192a);color:var(--text);font-family:Arial,Helvetica,sans-serif}
header{padding:22px 18px 16px;border-bottom:1px solid var(--line);background:#091626}
h1{font-size:clamp(21px,4vw,32px);letter-spacing:1px;margin:0;text-align:center}
.subtitle{text-align:center;color:var(--cyan);font-size:11px;letter-spacing:2px;margin-top:9px}
.members{text-align:center;color:var(--muted);font-size:11px;margin-top:10px;line-height:1.6}
.wrap{max-width:1200px;margin:auto;padding:16px}
.statusbar{display:flex;justify-content:space-between;align-items:center;gap:10px;flex-wrap:wrap;background:var(--panel);padding:13px 16px;border:1px solid var(--line);border-radius:12px;margin-bottom:14px}
.status{font-weight:bold;letter-spacing:1px}
.dot{display:inline-block;width:9px;height:9px;border-radius:50%;background:var(--green);margin-right:8px}
.alert .dot{background:var(--red);box-shadow:0 0 12px var(--red)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(155px,1fr));gap:12px}
.card{background:var(--panel);border:1px solid var(--line);border-radius:12px;padding:15px;min-width:0}
.label{color:var(--muted);font-size:11px;letter-spacing:1px;text-transform:uppercase}
.value{font-size:25px;font-weight:bold;margin-top:9px;overflow-wrap:anywhere}
.unit{font-size:12px;color:var(--muted);font-weight:normal}
.section{margin:20px 0 10px;font-size:13px;color:var(--cyan);letter-spacing:1.5px}
.charts{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px}
canvas{width:100%;height:190px;display:block}
.meta{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px}
.meta .card{padding:13px}
.meta .value{font-size:19px}
button{background:#123b52;border:1px solid #26708a;color:#e8f1fb;padding:10px 14px;border-radius:8px;font-weight:bold;cursor:pointer}
button:hover{background:#18516d}
.actions{display:flex;gap:10px;flex-wrap:wrap;margin-top:14px}
footer{text-align:center;color:#71869d;font-size:11px;padding:25px}
.small{font-size:11px;color:var(--muted)}
</style>
</head>
<body>
<header>
<h1>EARTHQUAKE MONITORING SYSTEM</h1>
<div class="subtitle">REAL-TIME SEISMIC ACTIVITY MONITORING &amp; ALERT SYSTEM</div>
<div class="members">DEVELOPED BY<br>Kushagra Agrawal (23EC29) &nbsp;|&nbsp; Rahul Dwivedi (23EC45) &nbsp;|&nbsp; Kapil Yadav (23EC27)</div>
</header>
<div class="wrap">
<div class="statusbar" id="statusbar">
<div class="status"><span class="dot"></span><span id="status">WAITING FOR TRANSMITTER</span></div>
<div class="small">Last packet age: <span id="age">--</span></div>
</div>

<div class="grid">
<div class="card"><div class="label">Acceleration X</div><div class="value"><span id="ax">--</span> <span class="unit">m/s²</span></div></div>
<div class="card"><div class="label">Acceleration Y</div><div class="value"><span id="ay">--</span> <span class="unit">m/s²</span></div></div>
<div class="card"><div class="label">Acceleration Z</div><div class="value"><span id="az">--</span> <span class="unit">m/s²</span></div></div>
<div class="card"><div class="label">RMS Vibration</div><div class="value"><span id="rms">--</span> <span class="unit">m/s²</span></div></div>
<div class="card"><div class="label">Peak Vibration</div><div class="value"><span id="peak">--</span> <span class="unit">m/s²</span></div></div>
<div class="card"><div class="label">Dominant Frequency</div><div class="value"><span id="frequency">--</span> <span class="unit">Hz</span></div></div>
</div>

<div class="section">SIGNAL &amp; SYSTEM TELEMETRY</div>
<div class="meta">
<div class="card"><div class="label">LoRa RSSI</div><div class="value"><span id="rssi">--</span> <span class="unit">dBm</span></div></div>
<div class="card"><div class="label">Estimated Distance</div><div class="value"><span id="distance">--</span> <span class="unit">m</span></div></div>
<div class="card"><div class="label">Packets Received</div><div class="value" id="packets">0</div></div>
<div class="card"><div class="label">Missed Packets</div><div class="value" id="missed">0</div></div>
<div class="card"><div class="label">Sequence Number</div><div class="value" id="sequence">--</div></div>
</div>

<div class="section">LIVE SIGNAL HISTORY</div>
<div class="charts">
<div class="card"><div class="label">Acceleration X / Y / Z</div><canvas id="accChart" width="600" height="240"></canvas></div>
<div class="card"><div class="label">RMS / Peak Vibration</div><canvas id="vibChart" width="600" height="240"></canvas></div>
<div class="card"><div class="label">Dominant Frequency</div><canvas id="freqChart" width="600" height="240"></canvas></div>
</div>

<div class="actions">
<button onclick="downloadCSV()">EXPORT SESSION CSV</button>
<button onclick="clearSession()">CLEAR SESSION DATA</button>
</div>
<p class="small">Distance is an estimate interpolated from your outdoor RSSI calibration points. Actual range varies with antenna orientation, obstacles and multipath.</p>
</div>
<footer>EARTHQUAKE MONITORING SYSTEM · LOCAL ESP8266 DASHBOARD</footer>

<script>
let rows=[];
let latest=null;
const $=id=>document.getElementById(id);
function fmt(v,n=2){return v===null||v===undefined||!Number.isFinite(Number(v))?'--':Number(v).toFixed(n)}
function setText(id,v){$(id).textContent=v}
function drawChart(id,series){
 const c=$(id),ctx=c.getContext('2d'),w=c.width,h=c.height;
 ctx.clearRect(0,0,w,h);
 const all=series.flatMap(s=>s.values).filter(Number.isFinite);
 if(!all.length){ctx.fillStyle='#91a6bd';ctx.fillText('Waiting for data...',18,35);return}
 let min=Math.min(0,...all),max=Math.max(...all);if(max-min<0.01){max=min+1}
 const pad=28,plotW=w-pad*2,plotH=h-pad*2;
 ctx.strokeStyle='#263c55';ctx.lineWidth=1;
 for(let i=0;i<5;i++){let y=pad+i*plotH/4;ctx.beginPath();ctx.moveTo(pad,y);ctx.lineTo(w-pad,y);ctx.stroke()}
 series.forEach(s=>{
  ctx.strokeStyle=s.color;ctx.lineWidth=2;ctx.beginPath();
  s.values.forEach((v,i)=>{
   let x=pad+(series[0].values.length<=1?0:i*plotW/(series[0].values.length-1));
   let y=pad+(max-v)*plotH/(max-min);
   if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);
  });ctx.stroke();
 });
 ctx.fillStyle='#91a6bd';ctx.font='11px Arial';
 ctx.fillText(max.toFixed(2),2,pad+4);ctx.fillText(min.toFixed(2),2,h-pad);
}
function updateCharts(h){
 drawChart('accChart',[
  {values:h.map(p=>p.x),color:'#46d9f2'},
  {values:h.map(p=>p.y),color:'#42df9b'},
  {values:h.map(p=>p.z),color:'#ffb454'}]);
 drawChart('vibChart',[
  {values:h.map(p=>p.rms),color:'#46d9f2'},
  {values:h.map(p=>p.peak),color:'#ff5e6c'}]);
 drawChart('freqChart',[{values:h.map(p=>p.freq),color:'#b89cff'}]);
}
async function refresh(){
 try{
  const r=await fetch('/data',{cache:'no-store'});const d=await r.json();latest=d;
  setText('status',d.status);
  $('statusbar').classList.toggle('alert',d.status==='ALERT');
  setText('age',d.connected?(d.age_ms/1000).toFixed(1)+' s':'No recent packet');
  ['ax','ay','az','rms','peak','frequency'].forEach(k=>setText(k,fmt(d[k])));
  setText('rssi',fmt(d.rssi,1));
  setText('distance',d.distance===null||d.distance<0?'OUT OF RANGE':fmt(d.distance,1));
  setText('packets',d.packets);setText('missed',d.missed);setText('sequence',d.sequence);
  updateCharts(d.history||[]);
  if(d.connected && d.history && d.history.length){
   const p=d.history[d.history.length-1];
   if(!rows.length || rows[rows.length-1].sequence!==d.sequence){
    rows.push({time:new Date().toISOString(),sequence:d.sequence,status:d.status,
      ax:d.ax,ay:d.ay,az:d.az,rms:d.rms,peak:d.peak,frequency:d.frequency,
      rssi:d.rssi,distance:d.distance});
   }
  }
 }catch(e){setText('status','DASHBOARD CONNECTION ERROR')}
}
function downloadCSV(){
 const headers=['time','sequence','status','ax_m_s2','ay_m_s2','az_m_s2','rms_m_s2','peak_m_s2','frequency_Hz','rssi_dBm','estimated_distance_m'];
 const lines=[headers.join(',')];
 rows.forEach(r=>lines.push(headers.map(k=>r[k]??'').join(',')));
 const blob=new Blob([lines.join('\n')],{type:'text/csv'});
 const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='earthquake_monitoring_session.csv';a.click();URL.revokeObjectURL(a.href);
}
function clearSession(){rows=[]}
refresh();setInterval(refresh,700);
</script>
</body>
</html>)=====";

// ---------------- HTTP ROUTES -----------------------
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  server.send(200, "application/json", buildDataJSON());
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ---------------- SETUP -----------------------------
void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // OLED I2C uses D2 SDA and D4 SCL.
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 initialization failed. Check OLED wiring/address."));
  } else {
    display.clearDisplay();
    display.display();
    startupStartTime = millis();
    startupSequenceActive = true;
    startupScreenShown = 255;
    drawStartupScreen(0);
    startupScreenShown = 0;
  }

  // Start local Wi-Fi access point.
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192,168,4,1),
                    IPAddress(192,168,4,1),
                    IPAddress(255,255,255,0));
  bool apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD, 6, false, 2);

  Serial.println();
  Serial.println(F("=== EARTHQUAKE MONITORING SYSTEM ==="));
  Serial.print(F("Wi-Fi AP: "));
  Serial.println(apStarted ? F("STARTED") : F("FAILED"));
  Serial.print(F("SSID: "));
  Serial.println(AP_SSID);
  Serial.print(F("Dashboard: http://"));
  Serial.println(WiFi.softAPIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.onNotFound(handleNotFound);
  server.begin();

  // Initialize SX1278.
  Serial.println(F("Initializing SX1278..."));
  int radioState = radio.begin(433.0);

  if (radioState == RADIOLIB_ERR_NONE) {
    radio.setOutputPower(10);
    radio.setDio0Action(setReceivedFlag, RISING);
    radio.startReceive();
    Serial.println(F("LoRa receiver ready at 433 MHz."));
  } else {
    Serial.print(F("LoRa initialization failed. Code: "));
    Serial.println(radioState);
    Serial.println(F("Check power, SPI wiring, NSS, RESET and DIO0."));
  }
}

// ---------------- MAIN LOOP -------------------------
void loop() {
  // Keep the web server responsive at all times.
  server.handleClient();

  // Service LoRa reception continuously, including during startup screens.
  serviceLoRa();

  // Update OLED startup sequence without blocking the receiver.
  updateStartupSequence();

  // Show live readings after the startup sequence has completed.
  if (!startupSequenceActive && millis() - lastDisplayMillis >= DISPLAY_REFRESH_MS) {
    lastDisplayMillis = millis();
    drawLiveOLED();
  }

  yield();
}
