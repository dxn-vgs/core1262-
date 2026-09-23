#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include <WiFi.h>
#include <WebServer.h>

// ESP32-WROOM -> Core1262-HF
static constexpr int PIN_SCK  = 18;
static constexpr int PIN_MISO = 19;
static constexpr int PIN_MOSI = 23;
static constexpr int PIN_NSS  = 21;
static constexpr int PIN_RST  = 22;
static constexpr int PIN_BUSY = 16;
static constexpr int PIN_DIO1 = 17;

#ifndef RANGE_ROLE_A
#define RANGE_ROLE_A 1
#endif
#ifndef RADIO_FREQ_MHZ
#define RADIO_FREQ_MHZ 869.0
#endif
#ifndef RADIO_BITRATE_KBPS
#define RADIO_BITRATE_KBPS 25.0
#endif
#ifndef RADIO_DEVIATION_KHZ
#define RADIO_DEVIATION_KHZ 25.0
#endif
#ifndef RADIO_RX_BW_KHZ
#define RADIO_RX_BW_KHZ 93.8
#endif
#ifndef RADIO_TX_POWER_DBM
#define RADIO_TX_POWER_DBM 22
#endif
#ifndef RADIO_PREAMBLE_BITS
#define RADIO_PREAMBLE_BITS 64
#endif
#ifndef EXCHANGE_INTERVAL_MS
#define EXCHANGE_INTERVAL_MS 1000
#endif

// Entire RF payload is exactly 2 bytes:
// [0] RSSI in signed dBm (int8_t)
// [1] random number 0..100
static constexpr size_t PACKET_SIZE = 2;
static constexpr int8_t RSSI_UNKNOWN = -127;
static constexpr size_t LOG_LINES = 80;

SPIClass radioSpi(VSPI);
SX1262 radio = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY, radioSpi);
WebServer server(80);

static constexpr char NODE_ID = RANGE_ROLE_A ? 'A' : 'B';
static constexpr const char* WIFI_SSID = RANGE_ROLE_A ? "ESP-A" : "ESP-B";
static constexpr const char* WIFI_PASSWORD = "core1262rx";

static volatile bool radioIrq = false;
static bool radioReady = false;
static int16_t radioInitCode = 0;

static float latestLocalRssi = -200.0f;
static int8_t latestPeerReportedRssi = RSSI_UNKNOWN;
static uint8_t latestPeerRandom = 0;
static uint8_t latestOwnRandom = 0;
static uint32_t txCount = 0;
static uint32_t rxCount = 0;
static uint32_t lastExchangeMs = 0;

static String logs[LOG_LINES];
static size_t logHead = 0;
static size_t logCount = 0;

static void onRadioIrq() {
  radioIrq = true;
}

static void addLog(const String& line) {
  Serial.println(line);
  logs[logHead] = line;
  logHead = (logHead + 1) % LOG_LINES;
  if (logCount < LOG_LINES) ++logCount;
}

static String logsJson() {
  String out = "[";
  for (size_t i = 0; i < logCount; ++i) {
    size_t idx = (logHead + LOG_LINES - logCount + i) % LOG_LINES;
    String s = logs[idx];
    s.replace("\\", "\\\\");
    s.replace("\"", "\\\"");
    if (i) out += ",";
    out += "\"" + s + "\"";
  }
  out += "]";
  return out;
}

static int8_t rssiForPayload() {
  if (latestLocalRssi <= -126.5f || latestLocalRssi >= 0.0f) return RSSI_UNKNOWN;
  int value = (int)roundf(latestLocalRssi);
  value = constrain(value, -126, -1);
  return (int8_t)value;
}

static int16_t startReceive() {
  radioIrq = false;
  return radio.startReceive();
}

static bool initRadio() {
  radioSpi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);

  radioInitCode = radio.beginFSK(
      RADIO_FREQ_MHZ,
      RADIO_BITRATE_KBPS,
      RADIO_DEVIATION_KHZ,
      RADIO_RX_BW_KHZ,
      RADIO_TX_POWER_DBM,
      RADIO_PREAMBLE_BITS);
  if (radioInitCode != RADIOLIB_ERR_NONE) return false;

  radioInitCode = radio.setCurrentLimit(140.0);
  if (radioInitCode != RADIOLIB_ERR_NONE) return false;

  radio.setPacketReceivedAction(onRadioIrq);
  radioInitCode = startReceive();
  if (radioInitCode != RADIOLIB_ERR_NONE) return false;

  radioReady = true;
  return true;
}

static bool transmitPayload() {
  uint8_t packet[PACKET_SIZE];
  const int8_t report = rssiForPayload();
  latestOwnRandom = (uint8_t)random(0, 101);
  packet[0] = (uint8_t)report;
  packet[1] = latestOwnRandom;

  radio.clearPacketReceivedAction();
  const int16_t txState = radio.transmit(packet, PACKET_SIZE);
  radio.setPacketReceivedAction(onRadioIrq);
  const int16_t rxState = startReceive();

  if (rxState != RADIOLIB_ERR_NONE) {
    addLog(String("[RX-START-ERR] code=") + rxState);
  }
  if (txState != RADIOLIB_ERR_NONE) {
    addLog(String("[TX-ERR] code=") + txState);
    return false;
  }

  ++txCount;
  addLog(String("[TX] RSSI=") + report + " dBm RAND=" + latestOwnRandom);
  return true;
}

static void processPacket() {
  if (!radioIrq) return;
  radioIrq = false;

  uint8_t packet[PACKET_SIZE] = {0};
  const int16_t state = radio.readData(packet, PACKET_SIZE);
  if (state != RADIOLIB_ERR_NONE) {
    addLog(String("[RX-ERR] code=") + state);
    startReceive();
    return;
  }

  latestLocalRssi = radio.getRSSI();
  latestPeerReportedRssi = (int8_t)packet[0];
  latestPeerRandom = packet[1];
  ++rxCount;

  addLog(String("[RX] measured=") + String(latestLocalRssi, 1) +
         " dBm peer_reported=" + latestPeerReportedRssi +
         " dBm RAND=" + latestPeerRandom);

  // Node B immediately answers every valid A packet with its own 2-byte payload.
  if (!RANGE_ROLE_A) {
    transmitPayload();
  } else {
    startReceive();
  }
}

static String statusJson() {
  String out = "{";
  out += "\"node\":\"" + String(NODE_ID) + "\"";
  out += ",\"ssid\":\"" + String(WIFI_SSID) + "\"";
  out += ",\"radioReady\":" + String(radioReady ? "true" : "false");
  out += ",\"radioInitCode\":" + String(radioInitCode);
  out += ",\"txPower\":" + String(RADIO_TX_POWER_DBM);
  out += ",\"localRssi\":" + String(latestLocalRssi, 1);
  out += ",\"peerReportedRssi\":" + String(latestPeerReportedRssi);
  out += ",\"ownRandom\":" + String(latestOwnRandom);
  out += ",\"peerRandom\":" + String(latestPeerRandom);
  out += ",\"txCount\":" + String(txCount);
  out += ",\"rxCount\":" + String(rxCount);
  out += "}";
  return out;
}

static const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP RSSI Exchange</title>
<style>
body{font-family:Arial,sans-serif;background:#111;color:#eee;margin:0;padding:18px}h1{font-size:22px;margin:0 0 6px}.small{font-size:12px;color:#aaa;margin-bottom:14px}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px;margin-bottom:14px}.card{background:#1c1c1c;border:1px solid #333;border-radius:10px;padding:14px}.k{font-size:12px;color:#999}.v{font-size:25px;margin-top:6px}.good{color:#49d17d}.bad{color:#ff6262}#logs{background:#050505;border:1px solid #333;border-radius:10px;padding:12px;height:42vh;overflow:auto;white-space:pre-wrap;font-family:monospace;font-size:13px}
</style></head><body>
<h1>Core1262-HF RSSI Exchange</h1><div class="small" id="meta">Loading...</div>
<div class="grid">
<div class="card"><div class="k">RADIO</div><div class="v" id="radio">-</div></div>
<div class="card"><div class="k">TX POWER</div><div class="v"><span id="power">-</span> dBm</div></div>
<div class="card"><div class="k">LOCAL MEASURED RSSI</div><div class="v"><span id="local">-</span> dBm</div></div>
<div class="card"><div class="k">PEER REPORTED RSSI</div><div class="v"><span id="peerRssi">-</span> dBm</div></div>
<div class="card"><div class="k">MY RANDOM</div><div class="v" id="myRand">-</div></div>
<div class="card"><div class="k">PEER RANDOM</div><div class="v" id="peerRand">-</div></div>
<div class="card"><div class="k">TX COUNT</div><div class="v" id="tx">0</div></div>
<div class="card"><div class="k">RX COUNT</div><div class="v" id="rx">0</div></div>
</div><div id="logs">Waiting...</div>
<script>
async function refresh(){try{const s=await fetch('/api/status').then(r=>r.json());meta.textContent='Node '+s.node+' | WiFi '+s.ssid+' | 869 MHz GFSK | 2-byte payload';radio.textContent=s.radioReady?'OK':'ERR '+s.radioInitCode;radio.className='v '+(s.radioReady?'good':'bad');power.textContent=s.txPower;local.textContent=s.localRssi;peerRssi.textContent=s.peerReportedRssi;myRand.textContent=s.ownRandom;peerRand.textContent=s.peerRandom;tx.textContent=s.txCount;rx.textContent=s.rxCount;const l=await fetch('/api/logs').then(r=>r.json());logs.textContent=l.join('\n');logs.scrollTop=logs.scrollHeight}catch(e){}}
setInterval(refresh,400);refresh();
</script></body></html>)HTML";

static void startWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  server.on("/", [](){ server.send_P(200, "text/html", PAGE); });
  server.on("/api/status", [](){ server.send(200, "application/json", statusJson()); });
  server.on("/api/logs", [](){ server.send(200, "application/json", logsJson()); });
  server.begin();
  addLog(String("[WEB] SSID=") + WIFI_SSID + " IP=" + WiFi.softAPIP().toString());
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  randomSeed((uint32_t)esp_random());
  startWeb();
  radioReady = initRadio();

  Serial.println();
  Serial.println("=== Core1262-HF RSSI + random exchange ===");
  Serial.printf("Node:          %c\n", NODE_ID);
  Serial.printf("WiFi:          %s\n", WIFI_SSID);
  Serial.printf("RF payload:    2 bytes (RSSI, random 0..100)\n");
  Serial.printf("Frequency:     %.3f MHz\n", (double)RADIO_FREQ_MHZ);
  Serial.printf("TX setting:    %d dBm (requested, not measured)\n", RADIO_TX_POWER_DBM);
  Serial.printf("Bitrate:       %.1f kbps\n", (double)RADIO_BITRATE_KBPS);

  if (!radioReady) {
    addLog(String("[RADIO] init failed code=") + radioInitCode);
  } else {
    addLog(String("[RADIO] Node ") + NODE_ID + " ready");
  }

  lastExchangeMs = millis();
}

void loop() {
  server.handleClient();
  if (!radioReady) { delay(2); return; }

  processPacket();

  // A drives the exchange once per second. B only replies.
  if (RANGE_ROLE_A) {
    const uint32_t now = millis();
    if ((uint32_t)(now - lastExchangeMs) >= EXCHANGE_INTERVAL_MS) {
      lastExchangeMs = now;
      transmitPayload();
    }
  }

  server.handleClient();
  delay(1);
}
