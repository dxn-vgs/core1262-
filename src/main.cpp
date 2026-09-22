#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

#if !RANGE_ROLE_TX
#include <WiFi.h>
#include <WebServer.h>
#endif

// ESP32-WROOM -> Core1262-HF
static constexpr int PIN_SCK  = 18;
static constexpr int PIN_MISO = 19;
static constexpr int PIN_MOSI = 23;
static constexpr int PIN_NSS  = 21;
static constexpr int PIN_RST  = 22;
static constexpr int PIN_BUSY = 16;
static constexpr int PIN_DIO1 = 17;

#ifndef RANGE_ROLE_TX
#define RANGE_ROLE_TX 0
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

static constexpr size_t PACKET_SIZE = 20;
static constexpr uint32_t TX_INTERVAL_MS = 1000;

SPIClass radioSpi(VSPI);
SX1262 radio = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY, radioSpi);

static uint32_t txSequence = 0;
static bool radioReady = false;
static int16_t radioInitCode = 0;

static void printHex(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    if (data[i] < 0x10) Serial.print('0');
    Serial.print(data[i], HEX);
    if (i + 1 < len) Serial.print(' ');
  }
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

  if (radioInitCode != RADIOLIB_ERR_NONE) {
    Serial.printf("[RADIO-INIT-ERROR] beginFSK failed: %d\n", radioInitCode);
    return false;
  }

  int16_t state = radio.setCurrentLimit(140.0);
  if (state != RADIOLIB_ERR_NONE) {
    radioInitCode = state;
    Serial.printf("[RADIO-INIT-ERROR] setCurrentLimit failed: %d\n", state);
    return false;
  }

  radioReady = true;
  Serial.println();
  Serial.println("=== Core1262-HF GFSK range test ===");
  Serial.printf("Role:          %s\n", RANGE_ROLE_TX ? "TRANSMITTER" : "RECEIVER");
  Serial.printf("Packet size:   %u bytes\n", (unsigned)PACKET_SIZE);
  Serial.printf("Frequency:     %.3f MHz\n", (double)RADIO_FREQ_MHZ);
  Serial.printf("TX power:      %d dBm\n", RADIO_TX_POWER_DBM);
  Serial.printf("Bitrate:       %.1f kbps\n", (double)RADIO_BITRATE_KBPS);
  Serial.printf("Deviation:     %.1f kHz\n", (double)RADIO_DEVIATION_KHZ);
  Serial.printf("RX bandwidth:  %.1f kHz\n", (double)RADIO_RX_BW_KHZ);
  Serial.printf("Preamble:      %d bits\n", RADIO_PREAMBLE_BITS);
  Serial.printf("OCP limit:     %.1f mA\n", (double)radio.getCurrentLimit());
  Serial.println();
  return true;
}

#if RANGE_ROLE_TX

static void buildPacket(uint8_t packet[PACKET_SIZE], uint32_t sequence) {
  packet[0] = (uint8_t)(sequence);
  packet[1] = (uint8_t)(sequence >> 8);
  packet[2] = (uint8_t)(sequence >> 16);
  packet[3] = (uint8_t)(sequence >> 24);

  static const uint8_t payload[16] = {
    'C','O','R','E','1','2','6','2',
    '-','G','F','S','K','-','2','2'
  };
  memcpy(packet + 4, payload, sizeof(payload));
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println("HELLO Core1262 TX firmware - one-side pin map v2");
  initRadio();
}

void loop() {
  if (!radioReady) {
    delay(1000);
    return;
  }

  uint8_t packet[PACKET_SIZE];
  buildPacket(packet, txSequence);

  const uint32_t started = millis();
  const int16_t state = radio.transmit(packet, PACKET_SIZE);
  const uint32_t elapsed = millis() - started;

  if (state == RADIOLIB_ERR_NONE) {
    Serial.printf("[TX] seq=%lu bytes=%u time=%lums data=",
                  (unsigned long)txSequence,
                  (unsigned)PACKET_SIZE,
                  (unsigned long)elapsed);
    printHex(packet, PACKET_SIZE);
    Serial.println();
  } else {
    Serial.printf("[TX-ERR] seq=%lu code=%d\n",
                  (unsigned long)txSequence, state);
  }

  ++txSequence;
  delay(TX_INTERVAL_MS);
}

#else

static constexpr const char* AP_SSID = "Core1262-RX";
static constexpr const char* AP_PASSWORD = "core1262rx";

WebServer server(80);

static constexpr size_t LOG_LINES = 80;
static String logs[LOG_LINES];
static size_t logHead = 0;
static size_t logCount = 0;

static uint32_t rxPackets = 0;
static uint32_t rxLost = 0;
static uint32_t lastSequence = 0;
static bool haveSequence = false;
static float latestRssi = -200.0f;
static uint32_t latestPacketMs = 0;
static String latestAscii = "";

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

static String statusJson() {
  const uint32_t expected = rxPackets + rxLost;
  const float per = expected ? (100.0f * rxLost / expected) : 0.0f;

  String out = "{";
  out += "\"radioReady\":" + String(radioReady ? "true" : "false");
  out += ",\"radioInitCode\":" + String(radioInitCode);
  out += ",\"rssi\":" + String(latestRssi, 1);
  out += ",\"received\":" + String(rxPackets);
  out += ",\"lost\":" + String(rxLost);
  out += ",\"per\":" + String(per, 2);
  out += ",\"lastSeq\":" + String(haveSequence ? lastSequence : 0);
  out += ",\"lastPacketMs\":" + String(latestPacketMs);
  String a = latestAscii;
  a.replace("\\", "\\\\");
  a.replace("\"", "\\\"");
  out += ",\"ascii\":\"" + a + "\"";
  out += "}";
  return out;
}

static const char PAGE[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Core1262 RX</title>
<style>
body{font-family:Arial,sans-serif;background:#111;color:#eee;margin:0;padding:18px}
h1{font-size:22px;margin:0 0 16px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;margin-bottom:14px}
.card{background:#1c1c1c;border:1px solid #333;border-radius:10px;padding:14px}
.k{font-size:12px;color:#999}.v{font-size:26px;margin-top:6px}
.good{color:#49d17d}.bad{color:#ff6262}
#logs{background:#050505;border:1px solid #333;border-radius:10px;padding:12px;height:48vh;overflow:auto;white-space:pre-wrap;font-family:monospace;font-size:13px}
.small{font-size:12px;color:#aaa;margin-bottom:12px}
</style>
</head>
<body>
<h1>Core1262-HF Receiver</h1>
<div class="small">ESP32 AP: Core1262-RX &nbsp; | &nbsp; 192.168.4.1</div>
<div class="grid">
  <div class="card"><div class="k">RADIO</div><div class="v" id="radio">-</div></div>
  <div class="card"><div class="k">RSSI</div><div class="v"><span id="rssi">-</span> dBm</div></div>
  <div class="card"><div class="k">RECEIVED</div><div class="v" id="rx">0</div></div>
  <div class="card"><div class="k">LOST</div><div class="v" id="lost">0</div></div>
  <div class="card"><div class="k">PER</div><div class="v"><span id="per">0</span>%</div></div>
  <div class="card"><div class="k">LAST SEQ</div><div class="v" id="seq">0</div></div>
</div>
<div class="card" style="margin-bottom:14px"><div class="k">LAST PAYLOAD</div><div id="ascii" style="font-family:monospace;margin-top:6px">-</div></div>
<div id="logs">Waiting for logs...</div>
<script>
async function refresh(){
  try{
    const s=await fetch('/api/status').then(r=>r.json());
    const ok=s.radioReady;
    const e=document.getElementById('radio');
    e.textContent=ok?'OK':'ERR '+s.radioInitCode;
    e.className='v '+(ok?'good':'bad');
    rssi.textContent=s.rssi;
    rx.textContent=s.received;
    lost.textContent=s.lost;
    per.textContent=s.per;
    seq.textContent=s.lastSeq;
    ascii.textContent=s.ascii||'-';
    const l=await fetch('/api/logs').then(r=>r.json());
    const box=document.getElementById('logs');
    box.textContent=l.join('\n');
    box.scrollTop=box.scrollHeight;
  }catch(e){}
}
setInterval(refresh,500); refresh();
</script>
</body>
</html>
)HTML";

static void startWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  server.on("/", []() {
    server.send_P(200, "text/html", PAGE);
  });
  server.on("/api/status", []() {
    server.send(200, "application/json", statusJson());
  });
  server.on("/api/logs", []() {
    server.send(200, "application/json", logsJson());
  });
  server.begin();

  addLog("HELLO Core1262 RX web firmware v3");
  addLog(String("[WEB] SSID=") + AP_SSID + " IP=" + WiFi.softAPIP().toString());
}

void setup() {
  Serial.begin(115200);
  delay(1200);

  startWeb();
  radioReady = initRadio();
  if (!radioReady) {
    addLog(String("[RADIO] init failed code=") + radioInitCode + " (web UI remains available)");
  } else {
    addLog("[RADIO] GFSK receiver ready");
  }
}

void loop() {
  server.handleClient();

  if (!radioReady) {
    delay(2);
    return;
  }

  uint8_t packet[PACKET_SIZE] = {0};
  const int16_t state = radio.receive(packet, PACKET_SIZE);

  server.handleClient();

  if (state == RADIOLIB_ERR_NONE) {
    const uint32_t sequence =
        ((uint32_t)packet[0]) |
        ((uint32_t)packet[1] << 8) |
        ((uint32_t)packet[2] << 16) |
        ((uint32_t)packet[3] << 24);

    if (haveSequence && sequence > lastSequence + 1) {
      rxLost += sequence - lastSequence - 1;
    }
    if (!haveSequence || sequence > lastSequence) {
      lastSequence = sequence;
      haveSequence = true;
    }

    ++rxPackets;
    latestRssi = radio.getRSSI();
    latestPacketMs = millis();

    latestAscii = "";
    for (size_t i = 4; i < PACKET_SIZE; ++i) {
      char c = (char)packet[i];
      latestAscii += (c >= 32 && c <= 126) ? c : '.';
    }

    const uint32_t expected = rxPackets + rxLost;
    const float per = expected ? (100.0f * rxLost / expected) : 0.0f;

    char line[180];
    snprintf(line, sizeof(line),
             "[RX] seq=%lu RSSI=%.1f dBm received=%lu lost=%lu PER=%.2f%% payload=%s",
             (unsigned long)sequence,
             (double)latestRssi,
             (unsigned long)rxPackets,
             (unsigned long)rxLost,
             (double)per,
             latestAscii.c_str());
    addLog(line);
  } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
    addLog(String("[RX-ERR] code=") + state);
  }
}

#endif
