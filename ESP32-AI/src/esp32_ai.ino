#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "kalman_filter.h"
#include "station_detector.h"
#include "adaptive_pid.h"
#include "map_memory.h"

// Hardware UART2 for Arduino communication
// GPIO16 = RX (receives from Arduino D1 / UNO TX)
// GPIO17 = TX (sends to Arduino D0 / UNO RX)
HardwareSerial ArduinoSerial(2);

#define WIFI_SSID "ESP32-Robot"
#define WIFI_PASS "robot1234"

WebServer server(80);

static float currentKP = 0.0f;
static float currentKD = 0.0f;
static uint8_t currentSpeed = 0;

#define EVENT_LOG_SIZE 16
#define CONF_HISTORY_SIZE 32
static String eventLog[EVENT_LOG_SIZE];
static uint8_t eventLogHead = 0;
static uint8_t eventLogCount = 0;
static float confidenceHistory[CONF_HISTORY_SIZE];
static uint8_t confidenceHistoryHead = 0;
static uint8_t confidenceHistoryCount = 0;

static String serialMessages[EVENT_LOG_SIZE];
static uint8_t serialMessagesHead = 0;
static uint8_t serialMessagesCount = 0;

// Command history (sent from web UI to Arduino/robot)
#define CMD_HISTORY_SIZE 8
static String cmdHistory[CMD_HISTORY_SIZE];
static bool   cmdConfirmed[CMD_HISTORY_SIZE];
static uint8_t cmdHead = 0;
static uint8_t cmdCount = 0;
static String lastCommandStr = "";
static bool   lastCommandConfirmed = false;

static const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>Robot Dashboard</title>
  <style>
    :root {
      --bg: #0a0a12;
      --panel: rgba(10, 14, 28, 0.95);
      --panel-soft: rgba(18, 24, 44, 0.92);
      --border: rgba(67, 198, 255, 0.18);
      --blue: #43c6ff;
      --green: #7efc56;
      --orange: #ffb33b;
      --red: #ff3f5c;
      --text: #e8f5ff;
      --muted: #7a8cae;
    }
    * { box-sizing: border-box; }
    body { margin: 0; min-height: 100vh; font-family: ui-monospace, SFMono-Regular, Consolas, 'Courier New', monospace; color: var(--text); background: radial-gradient(circle at top, rgba(67,198,255,0.08), transparent 25%), var(--bg); }
    header { padding: 24px 26px 10px; text-align: center; }
    h1 { margin: 0; font-size: 2.1rem; letter-spacing: 0.12em; color: #c8fbff; }
    p { margin: 6px 0 0; color: var(--muted); font-size: 0.95rem; }
    .page { display: grid; grid-template-columns: minmax(300px, 1fr) 1.4fr; gap: 20px; padding: 18px; }
    .panel { background: var(--panel); border: 1px solid rgba(255,255,255,0.08); border-radius: 24px; box-shadow: 0 0 32px rgba(67,198,255,0.08); overflow: hidden; }
    .panel-header { padding: 20px 22px; display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid rgba(255,255,255,0.08); }
    .panel-header h2 { margin: 0; font-size: 0.95rem; letter-spacing: 0.18em; text-transform: uppercase; color: var(--blue); }
    .status-pill { padding: 10px 14px; border-radius: 999px; background: rgba(67,198,255,0.14); color: #b7ebff; font-size: 0.85rem; }
    .grid-overview { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 16px; padding: 20px; }
    .stat-card { position: relative; overflow: hidden; padding: 20px; border-radius: 20px; background: rgba(6,10,18,0.95); border: 1px solid rgba(255,255,255,0.05); }
    .stat-title { margin: 0 0 10px; color: var(--muted); font-size: 0.82rem; text-transform: uppercase; letter-spacing: 0.14em; }
    .stat-value { margin: 0; font-size: 2rem; line-height: 1.05; }
    .stat-sub { margin-top: 8px; color: var(--muted); font-size: 0.84rem; }
    .gauge { height: 16px; margin-top: 16px; border-radius: 12px; overflow: hidden; background: rgba(255,255,255,0.06); }
    .gauge-fill { height: 100%; width: 0%; border-radius: 12px; background: var(--blue); box-shadow: 0 0 18px rgba(67,198,255,0.24); transition: width 0.35s ease, background 0.35s ease; }
    .chart-card { padding: 20px; display: grid; gap: 16px; }
    .chart-title { margin: 0; color: var(--muted); font-size: 0.88rem; text-transform: uppercase; letter-spacing: 0.16em; }
    #confidenceChart { width: 100%; height: 280px; border-radius: 20px; background: rgba(255,255,255,0.02); }
    .legend { display: flex; justify-content: space-between; align-items: center; color: var(--muted); font-size: 0.9rem; }
    .event-tabs { display: flex; gap: 10px; margin-bottom: 18px; }
    .tab-btn { flex: 1; padding: 12px 14px; border-radius: 16px; border: 1px solid rgba(255,255,255,0.08); background: rgba(255,255,255,0.03); color: var(--text); cursor: pointer; transition: background 0.2s ease; }
    .tab-btn.active { background: rgba(67,198,255,0.16); border-color: rgba(67,198,255,0.24); }
    .log-list { list-style: none; margin: 0; padding: 0 0 18px; max-height: 420px; overflow-y: auto; }
    .log-entry { display: grid; grid-template-columns: 1fr auto; gap: 12px; padding: 16px; margin-bottom: 12px; border-radius: 18px; background: rgba(255,255,255,0.02); border: 1px solid rgba(255,255,255,0.05); }
    .log-text { margin: 0; font-size: 0.95rem; color: var(--text); }
    .log-time { margin-top: 8px; color: var(--muted); font-size: 0.8rem; }
    .log-type { padding: 8px 12px; border-radius: 999px; text-transform: uppercase; letter-spacing: 0.16em; font-size: 0.75rem; min-width: 92px; text-align: center; }
    .log-confirm { background: rgba(126,252,86,0.14); color: var(--green); border: 1px solid rgba(126,252,86,0.18); }
    .log-ignore { background: rgba(255,140,62,0.14); color: var(--orange); border: 1px solid rgba(255,140,62,0.18); }
    .log-system { background: rgba(67,198,255,0.12); color: #8bd5ff; border: 1px solid rgba(67,198,255,0.2); }
    .map-table { width: 100%; border-collapse: collapse; }
    .map-table th, .map-table td { padding: 12px 14px; font-size: 0.92rem; }
    .map-table th { color: var(--muted); border-bottom: 1px solid rgba(255,255,255,0.08); }
    .map-table td { border-bottom: 1px solid rgba(255,255,255,0.04); }
    .footer { padding: 16px 0 24px; text-align: center; color: var(--muted); font-size: 0.9rem; }
    .footer strong { color: var(--blue); }
  </style>
</head>
<body>
  <header>
    <h1>CYBER ROVER COMMAND DECK</h1>
    <p>SoftAP active • connect to <strong>ESP32-Robot</strong> and browse <strong>192.168.4.1</strong></p>
    <p id="statusText" style="margin:8px 0 0; color: var(--muted); font-size:0.95rem;">Waiting for connection...</p>
    <div style="margin-top:10px; display:flex; gap:10px; justify-content:center;">
      <button class="tab-btn" style="min-width:120px;" onclick="sendCommand('stop')">Stop Robot</button>
      <button class="tab-btn" style="min-width:140px;" onclick="sendCommand('reset_arduino')">Reset Arduino</button>
      <button class="tab-btn" style="min-width:120px;" onclick="sendCommand('reset_esp')">Reset ESP</button>
    </div>
    <div style="margin-top:8px; text-align:center; color:var(--muted);">
      Last command: <strong id="lastCmd">--</strong> — <span id="lastCmdStatus">unknown</span>
    </div>
  </header>
  <div class="page">
    <div class="panel">
      <div class="panel-header">
        <h2>Telemetry Overview</h2>
        <span class="status-pill">AP Mode</span>
      </div>
      <div class="grid-overview">
        <div class="stat-card">
          <div class="stat-title">PID Gains</div>
          <div class="stat-value" id="pidValue">--</div>
          <div class="stat-sub">Real-time KP / KD tuning</div>
        </div>
        <div class="stat-card">
          <div class="stat-title">Drive Speed</div>
          <div class="stat-value" id="speedValue">--</div>
          <div class="stat-sub">Current motion setpoint</div>
        </div>
        <div class="stat-card">
          <div class="stat-title">Station Confidence</div>
          <div class="stat-value" id="confValue">--</div>
          <div class="gauge"><div class="gauge-fill" id="confGauge"></div></div>
          <div class="stat-sub" id="confLabel">Signal strength</div>
        </div>
        <div class="stat-card">
          <div class="stat-title">Last Station</div>
          <div class="stat-value" id="stationValue">0</div>
          <div class="stat-sub">Latest confirmed stop</div>
        </div>
      </div>
      <div class="chart-card">
        <div class="chart-title">Station Confidence Trend</div>
        <svg id="confidenceChart" viewBox="0 0 500 280" preserveAspectRatio="none">
          <defs>
            <linearGradient id="chartGradient" x1="0" y1="0" x2="0" y2="1">
              <stop offset="0%" stop-color="#43c6ff" stop-opacity="0.5" />
              <stop offset="100%" stop-color="#0a0a12" stop-opacity="0" />
            </linearGradient>
          </defs>
          <path id="chartFill" fill="url(#chartGradient)"></path>
          <polyline id="chartLine" fill="none" stroke="#43c6ff" stroke-width="3" stroke-linejoin="round" stroke-linecap="round"></polyline>
        </svg>
        <div class="legend"><span>Live station confidence</span><span id="chartPercent">--%</span></div>
      </div>
    </div>
    <div class="panel">
      <div class="panel-header">
        <h2>Event Log</h2>
        <small>Station confirmations, ignores, and system actions</small>
      </div>
      <div style="padding: 20px;">
        <div class="event-tabs">
          <button class="tab-btn active" id="tabOverview" onclick="showTab('overview')">Overview</button>
          <button class="tab-btn" id="tabLogs" onclick="showTab('logs')">Event Log</button>
        </div>
        <div id="overviewSection">
          <div class="stat-card" style="padding: 18px; background: rgba(255,255,255,0.02); border-color: rgba(67,198,255,0.08);">
            <div class="stat-title">Map segments</div>
            <table class="map-table" id="mapTable">
              <thead><tr><th>From</th><th>To</th><th>Avg ms</th><th>Samples</th></tr></thead>
              <tbody></tbody>
            </table>
          </div>
        </div>
        <div id="logsSection" style="display:none;">
          <ul class="log-list" id="eventList"></ul>
        </div>
      </div>
    </div>
    <div class="panel">
      <div class="panel-header">
        <h2>Serial Monitor</h2>
        <span class="status-pill">Live USB input</span>
      </div>
      <div style="padding: 20px;">
        <ul class="log-list" id="serialList"></ul>
      </div>
    </div>
  </div>
  <div class="footer">ESP32 SoftAP: <strong>ESP32-Robot</strong> • IP: <strong>192.168.4.1</strong></div>
  <script>
    const maxHistory = 32;
    function showTab(tab) {
      document.getElementById('overviewSection').style.display = tab === 'overview' ? 'block' : 'none';
      document.getElementById('logsSection').style.display = tab === 'logs' ? 'block' : 'none';
      document.getElementById('tabOverview').classList.toggle('active', tab === 'overview');
      document.getElementById('tabLogs').classList.toggle('active', tab === 'logs');
    }
    function getConfidenceColor(value) {
      if (value >= 0.8) return '#7efc56';
      if (value >= 0.5) return '#ffb33b';
      return '#ff3f5c';
    }
    function updateChart(values) {
      const width = 500;
      const height = 280;
      const padding = 20;
      const plotWidth = width - padding * 2;
      const plotHeight = height - padding * 2;
      if (!values || values.length === 0) {
        document.getElementById('chartLine').setAttribute('points', '');
        document.getElementById('chartFill').setAttribute('d', '');
        return;
      }
      const points = values.map((value, index) => {
        const x = padding + (plotWidth * index / (maxHistory - 1));
        const y = padding + plotHeight * (1 - value);
        return `${x},${y}`;
      }).join(' ');
      document.getElementById('chartLine').setAttribute('points', points);
      const fillPath = `M ${padding},${height-padding} L ${points} L ${padding+plotWidth},${height-padding} Z`;
      document.getElementById('chartFill').setAttribute('d', fillPath);
    }
    function renderEvents(events) {
      const list = document.getElementById('eventList');
      list.innerHTML = '';
      if (!events || events.length === 0) {
        const item = document.createElement('li');
        item.className = 'log-entry log-system';
        item.innerHTML = '<div><p class="log-text">No events yet</p></div><div class="log-type log-system">Idle</div>';
        list.appendChild(item);
        return;
      }
      events.forEach(evt => {
        const typeClass = evt.type === 'CONFIRM' ? 'log-confirm' : evt.type === 'IGNORE' ? 'log-ignore' : 'log-system';
        const item = document.createElement('li');
        item.className = 'log-entry';
        item.innerHTML = `<div><p class="log-text">${evt.message}</p><div class="log-time">${evt.time}</div></div><div class="log-type ${typeClass}">${evt.type}</div>`;
        list.appendChild(item);
      });
    }
    function renderSerialMessages(messages) {
      const list = document.getElementById('serialList');
      list.innerHTML = '';
      if (!messages || messages.length === 0) {
        const item = document.createElement('li');
        item.className = 'log-entry log-system';
        item.innerHTML = '<div><p class="log-text">No serial activity yet</p></div><div class="log-type log-system">Idle</div>';
        list.appendChild(item);
        return;
      }
      messages.forEach(msg => {
        const item = document.createElement('li');
        item.className = 'log-entry';
        item.innerHTML = `<div><p class="log-text">${msg}</p></div><div class="log-type log-system">Serial</div>`;
        list.appendChild(item);
      });
    }
    async function sendCommand(action) {
      try {
        const res = await fetch(`/command?action=${action}`);
        if (!res.ok) throw new Error('Command failed');
        const data = await res.json();
        document.getElementById('lastCmd').textContent = action;
        document.getElementById('lastCmdStatus').textContent = 'sent';
      } catch (err) {
        document.getElementById('lastCmdStatus').textContent = 'error';
      }
    }
    async function refresh() {
      try {
        const res = await fetch('/status');
        if (!res.ok) throw new Error('Fetch failed');
        const data = await res.json();
        document.getElementById('pidValue').textContent = `${data.kp.toFixed(1)} : ${data.kd.toFixed(1)}`;
        document.getElementById('speedValue').textContent = data.speed;
        const confPercent = Math.round(data.confidence * 100);
        document.getElementById('confValue').textContent = `${confPercent}%`;
        document.getElementById('stationValue').textContent = data.lastStation;
        const color = getConfidenceColor(data.confidence);
        document.getElementById('confGauge').style.width = `${confPercent}%`;
        document.getElementById('confGauge').style.background = color;
        document.getElementById('confLabel').textContent = confPercent >= 80 ? 'Stable station' : confPercent >= 50 ? 'Approaching' : 'Weak signal';
        document.getElementById('chartPercent').textContent = `${confPercent}%`;
        const tbody = document.querySelector('#mapTable tbody');
        tbody.innerHTML = '';
        data.mapSegments.forEach(seg => {
          const row = document.createElement('tr');
          row.innerHTML = `<td>${seg.from}</td><td>${seg.to}</td><td>${seg.avg}</td><td>${seg.samples}</td>`;
          tbody.appendChild(row);
        });
        updateChart(data.confidenceHistory || []);
        renderEvents(data.events || []);
        renderSerialMessages(data.serialMessages || []);
        // update last command info
        document.getElementById('lastCmd').textContent = data.lastCommand || '--';
        document.getElementById('lastCmdStatus').textContent = data.lastCommandConfirmed ? 'confirmed' : (data.lastCommand ? 'pending' : 'none');
        document.getElementById('speedValue').textContent = data.speed;
        document.getElementById('statusText').textContent = 'Live telemetry active';
      } catch (err) {
        document.getElementById('statusText').textContent = 'Connection error';
      }
    }
    setInterval(refresh, 1000);
    refresh();
  </script>
</body>
</html>
)rawliteral";

// Initial PID values (must match Arduino's starting values)
#define INIT_KP    30.0f
#define INIT_KD    220.0f
#define INIT_SPEED 85

// Journey tracking
static uint8_t  lastStation     = 0;
static uint32_t lastStationTime = 0;
// Periodic map print interval (ms)
#define MAP_PRINT_INTERVAL_MS 30000UL
static uint32_t lastMapPrintMs = 0;

// Forward declarations
void process_arduino_message(const char* msg);
void handleRoot();
void handleStatus();
void initWiFi();

void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

void handleCommand() {
  if (!server.hasArg("action")) {
    server.send(400, "application/json", "{\"ok\":false,\"error\":\"missing action\"}");
    return;
  }
  String action = server.arg("action");
  action.toLowerCase();
  if (action == "stop") {
    ArduinoSerial.println("CMD:STOP");
    pushPendingCommand("STOP");
    server.send(200, "application/json", "{\"ok\":true,\"action\":\"stop\"}");
    return;
  } else if (action == "reset_arduino") {
    ArduinoSerial.println("CMD:RESET");
    pushPendingCommand("RESET");
    server.send(200, "application/json", "{\"ok\":true,\"action\":\"reset_arduino\"}");
    return;
  } else if (action == "reset_esp") {
    server.send(200, "application/json", "{\"ok\":true,\"action\":\"reset_esp\"}");
    delay(50);
    ESP.restart();
    return;
  }
  // Unknown action
  server.send(400, "application/json", "{\"ok\":false,\"error\":\"unknown action\"}");
}

String jsonEscape(const String &raw) {
  String out;
  out.reserve(raw.length());
  for (size_t i = 0; i < raw.length(); i++) {
    char c = raw[i];
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default: out += c; break;
    }
  }
  return out;
}

void pushEvent(const char* type, const char* message) {
  String entry = "{";
  entry += "\"type\":\"" + String(type) + "\",";
  entry += "\"message\":\"" + jsonEscape(String(message)) + "\",";
  uint32_t seconds = millis() / 1000;
  uint32_t mins = seconds / 60;
  uint32_t secs = seconds % 60;
  char timeBuf[16];
  snprintf(timeBuf, sizeof(timeBuf), "%02u:%02u", (unsigned)mins, (unsigned)secs);
  entry += "\"time\":\"" + String(timeBuf) + "\"}";
  eventLog[eventLogHead] = entry;
  eventLogHead = (eventLogHead + 1) % EVENT_LOG_SIZE;
  if (eventLogCount < EVENT_LOG_SIZE) eventLogCount++;
}

void pushSerialMessage(const char* message) {
  serialMessages[serialMessagesHead] = String(message);
  serialMessagesHead = (serialMessagesHead + 1) % EVENT_LOG_SIZE;
  if (serialMessagesCount < EVENT_LOG_SIZE) serialMessagesCount++;
  // Debug: echo captured serial line back to USB serial for visibility
  Serial.print("[WEB] Captured Serial: ");
  Serial.println(message);
}

void pushPendingCommand(const char* cmd) {
  cmdHistory[cmdHead] = String(cmd);
  cmdConfirmed[cmdHead] = false;
  cmdHead = (cmdHead + 1) % CMD_HISTORY_SIZE;
  if (cmdCount < CMD_HISTORY_SIZE) cmdCount++;
  lastCommandStr = String(cmd);
  lastCommandConfirmed = false;
}

void addConfidenceSample(float value) {
  confidenceHistory[confidenceHistoryHead] = value;
  confidenceHistoryHead = (confidenceHistoryHead + 1) % CONF_HISTORY_SIZE;
  if (confidenceHistoryCount < CONF_HISTORY_SIZE) confidenceHistoryCount++;
}

void handleStatus() {
  adaptive_pid_get_values(&currentKP, &currentKD, &currentSpeed);
  float confidence = station_detector_get_confidence();
  addConfidenceSample(confidence);

  String json = "{";
  json += "\"wifiStatus\":\"AP mode\",";
  json += "\"kp\":" + String(currentKP, 1) + ",";
  json += "\"kd\":" + String(currentKD, 1) + ",";
  json += "\"speed\":" + String(currentSpeed) + ",";
  json += "\"confidence\":" + String(confidence, 2) + ",";
  json += "\"lastStation\":" + String(lastStation) + ",";
  json += "\"confidenceHistory\": [";
  for (uint8_t i = 0; i < confidenceHistoryCount; i++) {
    uint8_t idx = (confidenceHistoryHead + CONF_HISTORY_SIZE - confidenceHistoryCount + i) % CONF_HISTORY_SIZE;
    json += String(confidenceHistory[idx], 2);
    if (i + 1 < confidenceHistoryCount) json += ",";
  }
  json += "],";
  json += "\"events\": [";
  for (uint8_t i = 0; i < eventLogCount; i++) {
    uint8_t idx = (eventLogHead + EVENT_LOG_SIZE - eventLogCount + i) % EVENT_LOG_SIZE;
    json += eventLog[idx];
    if (i + 1 < eventLogCount) json += ",";
  }
  json += "],";
  json += "\"serialMessages\": [";
  for (uint8_t i = 0; i < serialMessagesCount; i++) {
    uint8_t idx = (serialMessagesHead + EVENT_LOG_SIZE - serialMessagesCount + i) % EVENT_LOG_SIZE;
    json += "\"" + jsonEscape(serialMessages[idx]) + "\"";
    if (i + 1 < serialMessagesCount) json += ",";
  }
  json += "],";
    // Include last command and command history
    json += "\"lastCommand\":\"" + jsonEscape(lastCommandStr) + "\",";
    json += "\"lastCommandConfirmed\":" + String(lastCommandConfirmed ? "true" : "false") + ",";
    json += "\"commands\": [";
    for (uint8_t i = 0; i < cmdCount; i++) {
      uint8_t idx = (cmdHead + CMD_HISTORY_SIZE - cmdCount + i) % CMD_HISTORY_SIZE;
      json += "{\"cmd\":\"" + jsonEscape(cmdHistory[idx]) + "\",\"confirmed\":" + String(cmdConfirmed[idx] ? "true" : "false") + "}";
      if (i + 1 < cmdCount) json += ",";
    }
    json += "],";
  json += "\"mapSegments\":";

  String mapJson;
  map_memory_get_json(mapJson);
  json += mapJson;
  json += "}";

  server.send(200, "application/json", json);
}

void initWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  delay(100);
  IPAddress ip = WiFi.softAPIP();
  Serial.print("WiFi AP started: ");
  Serial.print(WIFI_SSID);
  Serial.print(" / ");
  Serial.println(WIFI_PASS);
  Serial.print("Open http://");
  Serial.println(ip);
}

void setup() {
  Serial.begin(115200);
  ArduinoSerial.begin(9600, SERIAL_8N1, 16, 17);

  kalman_filter_init();
  station_detector_init();
  adaptive_pid_init(INIT_KP, INIT_KD, INIT_SPEED);
  map_memory_init();

  initWiFi();
  server.on("/", handleRoot);
  server.on("/command", handleCommand);
  server.on("/status", handleStatus);
  server.begin();

  Serial.println("ESP32 AI Co-processor ready");
}

void loop() {
  // Parse incoming lines from Arduino
  while (ArduinoSerial.available()) {
    static char    rxBuf[32];
    static uint8_t rxIdx = 0;

    char c = ArduinoSerial.read();
    if (c == '\n') {
      rxBuf[rxIdx] = '\0';
      process_arduino_message(rxBuf);
      rxIdx = 0;
    } else if (rxIdx < sizeof(rxBuf) - 1) {
      rxBuf[rxIdx++] = c;
    }
  }

  // Periodic adaptive PID analysis (internally throttled to 2000ms)
  adaptive_pid_update(ArduinoSerial);

  // Read incoming USB serial from the PC and display it on web dashboard
  static char usbBuf[128];
  static uint8_t usbIdx = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbIdx > 0) {
        usbBuf[usbIdx] = '\0';
        pushSerialMessage(usbBuf);
        usbIdx = 0;
      }
    } else if (usbIdx < sizeof(usbBuf) - 1) {
      usbBuf[usbIdx++] = c;
    }
  }

  // Handle incoming web client requests
  server.handleClient();

  // Periodically print stored map to USB Serial every MAP_PRINT_INTERVAL_MS
  if (millis() - lastMapPrintMs >= MAP_PRINT_INTERVAL_MS) {
    lastMapPrintMs = millis();
    map_memory_print_all();
  }
}

void process_arduino_message(const char* msg) {

  // Handle command acknowledgements from Arduino, format: ACK:CMD
  if (strncmp(msg, "ACK:", 4) == 0) {
    const char* ack = msg + 4;
    // find matching pending command and mark confirmed
    for (uint8_t i = 0; i < cmdCount; i++) {
      uint8_t idx = (cmdHead + CMD_HISTORY_SIZE - cmdCount + i) % CMD_HISTORY_SIZE;
      if (cmdHistory[idx].length() > 0 && cmdHistory[idx] == String(ack)) {
        cmdConfirmed[idx] = true;
        lastCommandConfirmed = true;
        pushEvent("CMD_ACK", ack);
        return;
      }
    }
    // If not found, still record as serial message
    pushSerialMessage(msg);
    return;
  }

  // Remote-received notice from Arduino: RCV:... (optional firmware support)
  if (strncmp(msg, "RCV:", 4) == 0) {
    pushEvent("REMOTE", msg + 4);
    pushSerialMessage(msg);
    return;
  }

  if (strncmp(msg, "IR:", 3) == 0 && strlen(msg) >= 7) {
    bool fl = (msg[3] == '1');
    bool cl = (msg[4] == '1');
    bool cr = (msg[5] == '1');
    bool fr = (msg[6] == '1');

    FilteredIR filtered = kalman_filter_update(fl, cl, cr, fr);
    station_detector_add_sample(filtered);

    float denom = filtered.farLeft + filtered.centerLeft +
                  filtered.centerRight + filtered.farRight;
    float error = 0.0f;
    if (denom > 0.01f) {
      error = (-3.0f * filtered.farLeft   +
               -1.0f * filtered.centerLeft +
               +1.0f * filtered.centerRight +
               +3.0f * filtered.farRight) / denom;
    }
    adaptive_pid_add_error(error);

    // ✨ NEW: Send updates after every IR frame (50ms)
    // Get current AI state
    float kp = 0.0f, kd = 0.0f;
    uint8_t spd = 0;
    adaptive_pid_get_values(&kp, &kd, &spd);
    
    // Get confidence and corrections
    float confidence = station_detector_get_confidence();
    bool improving = station_detector_is_improving();
    int8_t speedCorr = adaptive_pid_get_speed_correction();

    // Send PID update every frame
    char buf[48];
    snprintf(buf, sizeof(buf), "PID:%.1f:%.1f", kp, kd);
    ArduinoSerial.println(buf);

    // Send confidence level if approaching station
    if (confidence > 0.5f) {
      snprintf(buf, sizeof(buf), "CONF:%.2f", confidence);
      ArduinoSerial.println(buf);
    }

    // Send speed correction if needed
    if (speedCorr != 0) {
      snprintf(buf, sizeof(buf), "SPD:%d", (int)spd + speedCorr);
      ArduinoSerial.println(buf);
    }

    // Record confidence history for the dashboard
    addConfidenceSample(confidence);

    // Per-frame PID adaptation
    adaptive_pid_update_per_frame(ArduinoSerial);

  } else if (strcmp(msg, "MAP:PRINT") == 0) {
    // Print stored map to USB Serial (ESP32) when requested by Arduino
    map_memory_print_all();

  } else if (strncmp(msg, "DST:", 4) == 0) {
    float dist = atof(msg + 4);
    // Reserved for future obstacle analysis
    (void)dist;

  } else if (strncmp(msg, "STN:DETECT", 10) == 0) {
    // Evaluate collected history and send CONFIRM/IGNORE back to Arduino.
    bool confirmed = station_detector_evaluate(ArduinoSerial);

    uint32_t now = millis();
    if (confirmed) {
      if (lastStationTime > 0) {
        uint32_t travelMs = now - lastStationTime;
        Serial.print("[MAIN] Travel time: ");
        Serial.print(travelMs);
        Serial.println("ms");

        uint8_t nextStation = lastStation + 1;
        // Record map segment (from lastStation -> nextStation)
        map_memory_record(lastStation, nextStation, travelMs);

        // Compare against expected and warn if deviant (>30%)
        uint32_t expected = map_memory_expected_ms(lastStation, nextStation);
        if (expected > 0) {
          float ratio = (float)travelMs / (float)expected;
          if (ratio > 1.3f || ratio < 0.7f) {
            Serial.print("[MAP] Travel time deviates from expected (exp=");
            Serial.print(expected);
            Serial.print("ms) got=");
            Serial.print(travelMs);
            Serial.println("ms — possible sync/obstacle issue");
          }
        }

        lastStation = nextStation;
      }
      lastStationTime = now;
    } else {
      // Not confirmed — update lastStationTime so next confirmed segment measures correctly
      lastStationTime = now;
    }
  }
}
