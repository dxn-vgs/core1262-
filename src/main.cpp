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
#ifndef PING_INTERVAL_MS
#define PING_INTERVAL_MS 1000
#endif
#ifndef REPLY_TIMEOUT_MS
#define REPLY_TIMEOUT_MS 600
#endif

static constexpr uint8_t PROTOCOL_MAGIC = 0xC2;
static constexpr uint8_t PROTOCOL_VERSION = 1;
static constexpr uint8_t TYPE_PING = 1;
static constexpr uint8_t TYPE_REPLY = 2;
static constexpr size_t PACKET_SIZE = 20;
static constexpr size_t LOG_LINES = 80;

SPIClass radioSpi(VSPI);
SX1262 radio = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY, radioSpi);
WebServer server(80);

static volatile bool radioIrq = false;
static bool radioReady = false;
static int16_t radioInitCode = 0;

static uint32_t txSequence = 0;
static uint32_t lastPeerSequence = 0;
static bool havePeerSequence = false;
static uint32_t txPackets = 0;
static uint32_t rxPackets = 0;
static uint32_t lostPackets = 0;
static uint32_t timeoutPackets = 0;
static uint32_t lastPingSentMs = 0;
static uint32_t pendingPingSeq = 0;
static uint32_t pendingPingStartedMs = 0;
static bool pingPending = false;
static float latestLocalRssi = -200.0f;
static float latestRemoteRssi = -200.0f;
static uint16_t latestRttMs = 0;
static uint32_t latestPacketMs = 0;

static String logs[LOG_LINES];
static size_t logHead = 0;
static size_t logCount = 0;

static constexpr const char* ROLE_NAME = RANGE_ROLE_A ? "A" : "B";
static constexpr uint8_t ROLE_ID = RANGE_ROLE_A ? 1 : 2;
static constexpr const char* AP_PASSWORD = "core1262rx";

static void onRadioIrq() {
  radioIrq = true;
}

static uint32_t readU32(const uint8_t* p) {
  return (uint32_t)p[0] |
         ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static void writeU32(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

static int16_t readI16(const uint8_t* p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void writeI16(uint8_t* p, int16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)((uint16_t)v >> 8);
}

static uint16_t readU16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void writeU16(uint8_t* p, uint16_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
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
    s.replace("\n", "\\n");
    if (i) out += ",";
    out += "\"" + s + "\"";
  }
  out += "]";
  return out;
}

static bool validPacket(const uint8_t* p) {
  return p[0] == PROTOCOL_MAGIC &&
         p[3] == PROTOCOL_VERSION &&
         p[16] == '1' && p[17] == '2' && p[18] == '6' && p[19] == '2';
}

static void buildPacket(uint8_t* p, uint8_t type, uint32_t sequence, uint32_t stampMs) {
  memset(p, 0, PACKET_SIZE);
  p[0] = PROTOCOL_MAGIC;
  p[1] = type;
  p[2] = ROLE_ID;
  p[3] = PROTOCOL_VERSION;
  writeU32(p + 4, sequence);
  writeU32(p + 8, stampMs);
  const int16_t rssiX10 = latestLocalRssi > -199.0f ? (int16_t)(latestLocalRssi * 10.0f) : INT16_MIN;
  writeI16(p + 12, rssiX10);
  writeU16(p + 14, latestRttMs);
  p[16] = '1'; p[17] = '2'; p[18] = '6'; p[19] = '2';
}

static void updatePeerSequence(uint32_t sequence) {
  if (havePeerSequence && sequence > lastPeerSequence + 1) {
    lostPackets += sequence - lastPeerSequence - 1;
  }
  if (!havePeerSequence || sequence > lastPeerSequence) {
    lastPeerSequence = sequence;
    havePeerSequence = true;
  }
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
  int16_t state = radio.setCurrentLimit(140.0);
  if (state != RADIOLIB_ERR_NONE) {
    radioInitCode = state;
    return false;
  }
  radio.setPacketReceivedAction(onRadioIrq);
  state = startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    radioInitCode = state;
    return false;
  }
  radioReady = true;
  return true;
}

static bool transmitPacket(uint8_t type, uint32_t sequence, uint32_t stampMs) {
  uint8_t packet[PACKET_SIZE];
  buildPacket(packet, type, sequence, stampMs);
  radio.clearPacketReceivedAction();
  const int16_t state = radio.transmit(packet, PACKET_SIZE);
  radio.setPacketReceivedAction(onRadioIrq);
  const int16_t rxState = startReceive();
  if (state == RADIOLIB_ERR_NONE) ++txPackets;
  if (rxState != RADIOLIB_ERR_NONE) {
    addLog(String("[RX-START-ERR] code=") + rxState);
  }
  if (state != RADIOLIB_ERR_NONE) {
    addLog(String("[TX-ERR] code=") + state);
    return false;
  }
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
  if (!validPacket(packet)) {
    addLog("[RX] ignored incompatible packet");
    startReceive();
    return;
  }

  const uint8_t type = packet[1];
  const uint8_t senderRole = packet[2];
  const uint32_t sequence = readU32(packet + 4);
  const uint32_t stampMs = readU32(packet + 8);
  const int16_t peerReportedRssiX10 = readI16(packet + 12);
  const uint16_t peerReportedRtt = readU16(packet + 14);

  latestLocalRssi = radio.getRSSI();
  latestPacketMs = millis();
  ++rxPackets;
  if (peerReportedRssiX10 != INT16_MIN) {
    latestRemoteRssi = (float)peerReportedRssiX10 / 10.0f;
  }
  if (!RANGE_ROLE_A && peerReportedRtt > 0) {
    latestRttMs = peerReportedRtt;
  }

  if (RANGE_ROLE_A && type == TYPE_REPLY && senderRole == 2) {
    if (pingPending && sequence == pendingPingSeq) {
      latestRttMs = (uint16_t)min<uint32_t>(millis() - pendingPingStartedMs, 65535);
      pingPending = false;
    }
    char line[180];
    snprintf(line, sizeof(line),
             "[REPLY] seq=%lu B->A=%.1f dBm A->B=%.1f dBm RTT=%u ms",
             (unsigned long)sequence,
             (double)latestLocalRssi,
             (double)latestRemoteRssi,
             latestRttMs);
    addLog(line);
  } else if (!RANGE_ROLE_A && type == TYPE_PING && senderRole == 1) {
    updatePeerSequence(sequence);
    char line[180];
    snprintf(line, sizeof(line),
             "[PING] seq=%lu A->B=%.1f dBm B->A=%.1f dBm lastRTT=%u ms",
             (unsigned long)sequence,
             (double)latestLocalRssi,
             (double)latestRemoteRssi,
             latestRttMs);
    addLog(line);
    transmitPacket(TYPE_REPLY, sequence, stampMs);
  } else {
    startReceive();
  }
}

static String statusJson() {
  const uint32_t expected = rxPackets + lostPackets + timeoutPackets;
  const float loss = expected ? (100.0f * (lostPackets + timeoutPackets) / expected) : 0.0f;
  String out = "{";
  out += "\"role\":\"" + String(ROLE_NAME) + "\"";
  out += ",\"radioReady\":" + String(radioReady ? "true" : "false");
  out += ",\"radioInitCode\":" + String(radioInitCode);
  out += ",\"localRssi\":" + String(latestLocalRssi, 1);
  out += ",\"remoteRssi\":" + String(latestRemoteRssi, 1);
  out += ",\"rttMs\":" + String(latestRttMs);
  out += ",\"tx\":" + String(txPackets);
  out += ",\"rx\":" + String(rxPackets);
  out += ",\"lost\":" + String(lostPackets + timeoutPackets);
  out += ",\"lossPct\":" + String(loss, 2);
  out += ",\"lastPacketMs\":" + String(latestPacketMs);
  out += ",\"pending\":" + String(pingPending ? "true" : "false");
  out += "}";
  return out;
}

static const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Core1262 Link Test</title><style>
body{font-family:Arial,sans-serif;background:#111;color:#eee;margin:0;padding:18px}h1{font-size:22px;margin:0 0 6px}.small{font-size:12px;color:#aaa;margin-bottom:14px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(145px,1fr));gap:10px;margin-bottom:14px}.card{background:#1c1c1c;border:1px solid #333;border-radius:10px;padding:14px}.k{font-size:12px;color:#999}.v{font-size:25px;margin-top:6px}.good{color:#49d17d}.bad{color:#ff6262}#logs{background:#050505;border:1px solid #333;border-radius:10px;padding:12px;height:44vh;overflow:auto;white-space:pre-wrap;font-family:monospace;font-size:13px}
</style></head><body><h1>Core1262-HF Two-Way Link Test</h1><div class="small" id="meta">Loading...</div>
<div class="grid"><div class="card"><div class="k">RADIO</div><div class="v" id="radio">-</div></div><div class="card"><div class="k">LOCAL RX RSSI</div><div class="v"><span id="local">-</span> dBm</div></div><div class="card"><div class="k">REMOTE RX RSSI</div><div class="v"><span id="remote">-</span> dBm</div></div><div class="card"><div class="k">RTT</div><div class="v"><span id="rtt">0</span> ms</div></div><div class="card"><div class="k">TX</div><div class="v" id="tx">0</div></div><div class="card"><div class="k">RX</div><div class="v" id="rx">0</div></div><div class="card"><div class="k">LOST</div><div class="v" id="lost">0</div></div><div class="card"><div class="k">LOSS</div><div class="v"><span id="loss">0</span>%</div></div></div>
<div id="logs">Waiting for packets...</div><script>
async function refresh(){try{const s=await fetch('/api/status').then(r=>r.json());meta.textContent='Role '+s.role+' | 869 MHz GFSK | AP Core1262-'+s.role+' | 192.168.4.1';radio.textContent=s.radioReady?'OK':'ERR '+s.radioInitCode;radio.className='v '+(s.radioReady?'good':'bad');local.textContent=s.localRssi;remote.textContent=s.remoteRssi;rtt.textContent=s.rttMs;tx.textContent=s.tx;rx.textContent=s.rx;lost.textContent=s.lost;loss.textContent=s.lossPct;const l=await fetch('/api/logs').then(r=>r.json());logs.textContent=l.join('\n');logs.scrollTop=logs.scrollHeight}catch(e){}}setInterval(refresh,500);refresh();
</script></body></html>)HTML";

static void startWeb() {
  const String ssid = String("Core1262-") + ROLE_NAME;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid.c_str(), AP_PASSWORD);
  server.on("/", [](){ server.send_P(200, "text/html", PAGE); });
  server.on("/api/status", [](){ server.send(200, "application/json", statusJson()); });
  server.on("/api/logs", [](){ server.send(200, "application/json", logsJson()); });
  server.begin();
  addLog(String("[WEB] SSID=") + ssid + " IP=" + WiFi.softAPIP().toString());
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  startWeb();
  radioReady = initRadio();
  Serial.println();
  Serial.println("=== Core1262-HF two-way range test ===");
  Serial.printf("Role:          Node %s\n", ROLE_NAME);
  Serial.printf("Frequency:     %.3f MHz\n", (double)RADIO_FREQ_MHZ);
  Serial.printf("TX setting:    %d dBm (requested, not measured)\n", RADIO_TX_POWER_DBM);
  Serial.printf("Bitrate:       %.1f kbps\n", (double)RADIO_BITRATE_KBPS);
  Serial.printf("Deviation:     %.1f kHz\n", (double)RADIO_DEVIATION_KHZ);
  Serial.printf("RX bandwidth:  %.1f kHz\n", (double)RADIO_RX_BW_KHZ);
  Serial.printf("Preamble:      %d bits\n", RADIO_PREAMBLE_BITS);
  Serial.printf("OCP limit:     %.1f mA\n", (double)radio.getCurrentLimit());
  if (!radioReady) addLog(String("[RADIO] init failed code=") + radioInitCode);
  else addLog(String("[RADIO] Node ") + ROLE_NAME + " ready");
}

void loop() {
  server.handleClient();
  if (!radioReady) { delay(2); return; }

  processPacket();

  if (RANGE_ROLE_A) {
    const uint32_t now = millis();
    if (pingPending && (uint32_t)(now - pendingPingStartedMs) >= REPLY_TIMEOUT_MS) {
      ++timeoutPackets;
      pingPending = false;
      addLog(String("[TIMEOUT] seq=") + pendingPingSeq);
    }
    if (!pingPending && (uint32_t)(now - lastPingSentMs) >= PING_INTERVAL_MS) {
      const uint32_t seq = txSequence++;
      pendingPingSeq = seq;
      pendingPingStartedMs = now;
      lastPingSentMs = now;
      pingPending = transmitPacket(TYPE_PING, seq, now);
      if (pingPending) {
        addLog(String("[PING-TX] seq=") + seq);
      }
    }
  }

  server.handleClient();
  delay(1);
}
