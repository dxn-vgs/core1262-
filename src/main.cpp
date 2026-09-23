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
#define REPLY_TIMEOUT_MS 350
#endif

// Keep the old on-air length: 20 bytes total.
// [0] sender ('A'/'B'), [1..3] sequence U24LE, [4..19] 16-byte message.
static constexpr size_t PACKET_SIZE = 20;
static constexpr size_t MESSAGE_SIZE = 16;
static constexpr size_t LOG_LINES = 80;
static constexpr uint32_t NODE_B_OFFSET_MS = PING_INTERVAL_MS / 2;

SPIClass radioSpi(VSPI);
SX1262 radio = new Module(PIN_NSS, PIN_DIO1, PIN_RST, PIN_BUSY, radioSpi);
WebServer server(80);

static volatile bool radioIrq = false;
static bool radioReady = false;
static int16_t radioInitCode = 0;

static constexpr char NODE_ID = RANGE_ROLE_A ? 'A' : 'B';
static constexpr char PEER_ID = RANGE_ROLE_A ? 'B' : 'A';
static constexpr const char* WIFI_SSID = RANGE_ROLE_A ? "ESP-A" : "ESP-B";
static constexpr const char* WIFI_PASSWORD = "core1262rx";

// Each word is normalized to exactly 16 bytes on air: truncate if long, '_' pad if short.
static const char* const MESSAGE_POOL[] = {
  "Bamboozlementing", "Brouhahahahahaaa", "Cattywampusified", "Codswalloperying",
  "Confusticationed", "Flummoxification", "Higgledypiggledy", "Hocuspocusifying",
  "Kerfufflemakers", "Malarkeyization", "Mumbojumboifying", "Rigamaroleifying",
  "Shenaniganizers", "Skedaddledoodler", "Snafuificationing", "Squeegeification",
  "Absentmindedness", "Babooneryshiness", "Befuddlementness", "Bootlickeriously",
  "Curmudgeonlyhood", "Featherbrainedly", "Fuddyuddyistical", "Giddyheadednesss",
  "Harebrainedlynes", "Highfalutinismus", "Knowitallishness", "Nincompooperyism",
  "Nincompoopshines", "Ninnyhammeringly", "Poppycockishness", "Rapscallionerise",
  "Scallywagfulness", "Scatterbrainedly", "Sillybillynesses", "Smartypantsified",
  "Snollygosterisms", "Toadeaterishness", "Tomfoolerymaking", "Whippersnapperin",
  "Wiseackerinesses", "Baublecollection", "Blunderbussingly", "Doohickeygizmoes",
  "Frillsandfurbelo", "Gewgawifications", "Gimcrackerything", "Jackolancanthorn",
  "Kickshawsbaking", "Knickknackerying", "Pumpernickeling", "Snickersneeblade",
  "Tchotchkeseeker", "Thingamajigified", "Whatchamacallits", "Balderdashedness",
  "Boondogglingwise", "Bumbazlementings", "Claptrapitations", "Cockalorumnesses",
  "Conundrumalities", "Coxcombicalities", "Fiddlestickingly", "Flabbergastingli",
  "Flusteratednesss", "Folderollednesse", "Gerrymandererism", "Gibberishmongers",
  "Gobbledygookers", "Hodgepodgeriness", "Hornswogglerisms", "Lickspittlerling",
  "Pettifoggerizing", "Poltrooneryships", "Razzledazzleries", "Skullduggeriests",
  "Taradiddleheaded", "Castlesintheairy", "Chimericalnesses", "Cloudcuckoolands",
  "Daydreamingfully", "Donquixoteishnes", "Flashinthepaning", "Flybynightnesses",
  "Forgetfulnesses!", "Gargantuanlylazy", "Gaudinessesfully", "Ignisfatuuslight",
  "Meretriciousness", "Moonshinehunters", "Nambypambyismsy", "Phantasmagorical",
  "Quixoticalnesses", "Quizzicalityness", "Reverieishnesses", "Tawdrinessesover",
  "Utopianistifying", "Willothewispings"
};
static constexpr size_t MESSAGE_COUNT = sizeof(MESSAGE_POOL) / sizeof(MESSAGE_POOL[0]);

static uint32_t ownSequence = 1;
static uint32_t ownGenerated = 0;
static uint32_t echoConfirmed = 0;
static uint32_t echoMissed = 0;
static uint32_t badEcho = 0;
static uint32_t duplicatePeer = 0;

static uint32_t peerSeqMax = 0;
static uint32_t peerReceived = 0;
static uint32_t peerMissed = 0;

static uint8_t pendingPacket[PACKET_SIZE] = {0};
static bool pendingEcho = false;
static uint32_t pendingStartedMs = 0;
static uint32_t nextOwnSendMs = 0;

static float latestRssi = -200.0f;
static uint16_t latestRttMs = 0;
static String lastSentMessage = "-";
static String lastReceivedMessage = "-";
static String lastEchoMessage = "-";
static bool lastEchoMatched = false;

static String logs[LOG_LINES];
static size_t logHead = 0;
static size_t logCount = 0;

static void onRadioIrq() { radioIrq = true; }

static uint32_t readU24(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

static void writeU24(uint8_t* p, uint32_t v) {
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
}

static String messageString(const uint8_t* packet) {
  char text[MESSAGE_SIZE + 1];
  memcpy(text, packet + 4, MESSAGE_SIZE);
  text[MESSAGE_SIZE] = '\0';
  return String(text);
}

static void normalizeMessage(const char* source, uint8_t* out) {
  memset(out, '_', MESSAGE_SIZE);
  const size_t n = min(strlen(source), MESSAGE_SIZE);
  memcpy(out, source, n);
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

static int16_t startReceive() {
  radioIrq = false;
  return radio.startReceive();
}

static bool initRadio() {
  radioSpi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);
  radioInitCode = radio.beginFSK(
      RADIO_FREQ_MHZ, RADIO_BITRATE_KBPS, RADIO_DEVIATION_KHZ,
      RADIO_RX_BW_KHZ, RADIO_TX_POWER_DBM, RADIO_PREAMBLE_BITS);
  if (radioInitCode != RADIOLIB_ERR_NONE) return false;

  radioInitCode = radio.setCurrentLimit(140.0);
  if (radioInitCode != RADIOLIB_ERR_NONE) return false;

  radio.setPacketReceivedAction(onRadioIrq);
  radioInitCode = startReceive();
  if (radioInitCode != RADIOLIB_ERR_NONE) return false;

  radioReady = true;
  return true;
}

static bool transmitRaw(const uint8_t* packet) {
  radio.clearPacketReceivedAction();
  const int16_t txState = radio.transmit((uint8_t*)packet, PACKET_SIZE);
  radio.setPacketReceivedAction(onRadioIrq);
  const int16_t rxState = startReceive();
  if (rxState != RADIOLIB_ERR_NONE) addLog(String("[RX-START-ERR] ") + rxState);
  if (txState != RADIOLIB_ERR_NONE) {
    addLog(String("[TX-ERR] ") + txState);
    return false;
  }
  return true;
}

static void buildOwnPacket(uint8_t* packet, uint32_t sequence) {
  packet[0] = (uint8_t)NODE_ID;
  writeU24(packet + 1, sequence);
  const char* selected = MESSAGE_POOL[random(MESSAGE_COUNT)];
  normalizeMessage(selected, packet + 4);
}

static void updatePeerSequence(uint32_t sequence) {
  if (sequence == 0) return;

  if (peerSeqMax == 0) {
    peerSeqMax = sequence;
    peerReceived = 1;
    peerMissed = sequence - 1;
    return;
  }

  if (sequence > peerSeqMax) {
    peerMissed += sequence - peerSeqMax - 1;
    peerSeqMax = sequence;
    ++peerReceived;
  } else {
    ++duplicatePeer;
  }
}

static void sendOwnPacket() {
  if (pendingEcho) return;

  const uint32_t sequence = ownSequence++;
  buildOwnPacket(pendingPacket, sequence);
  lastSentMessage = messageString(pendingPacket);

  ++ownGenerated;
  pendingStartedMs = millis();
  pendingEcho = transmitRaw(pendingPacket);

  addLog(String("[TX NEW] seq=") + sequence + " msg=" + lastSentMessage);
}

static void processPacket() {
  if (!radioIrq) return;
  radioIrq = false;

  uint8_t packet[PACKET_SIZE] = {0};
  const int16_t state = radio.readData(packet, PACKET_SIZE);
  if (state != RADIOLIB_ERR_NONE) {
    addLog(String("[RX-ERR] ") + state);
    startReceive();
    return;
  }

  const char sender = (char)packet[0];
  const uint32_t sequence = readU24(packet + 1);
  latestRssi = radio.getRSSI();

  if (sender == PEER_ID) {
    updatePeerSequence(sequence);
    lastReceivedMessage = messageString(packet);
    addLog(String("[RX NEW] from=") + sender + " seq=" + sequence +
           " msg=" + lastReceivedMessage + " RSSI=" + String(latestRssi, 1) + " dBm");

    // Echo exactly what was received, byte-for-byte.
    transmitRaw(packet);
    addLog(String("[ECHO TX] seq=") + sequence + " msg=" + lastReceivedMessage);
    return;
  }

  if (sender == NODE_ID) {
    lastEchoMessage = messageString(packet);
    const bool same = pendingEcho && memcmp(packet, pendingPacket, PACKET_SIZE) == 0;

    if (same) {
      latestRttMs = (uint16_t)min<uint32_t>(millis() - pendingStartedMs, 65535);
      ++echoConfirmed;
      lastEchoMatched = true;
      pendingEcho = false;
      addLog(String("[ECHO OK] seq=") + sequence + " msg=" + lastEchoMessage +
             " RTT=" + latestRttMs + " ms RSSI=" + String(latestRssi, 1) + " dBm");
    } else {
      ++badEcho;
      lastEchoMatched = false;
      addLog(String("[ECHO BAD] seq=") + sequence + " got=" + lastEchoMessage);
    }
    return;
  }

  addLog(String("[RX UNKNOWN] sender=") + sender);
  startReceive();
}

static String statusJson() {
  const float peerLossPct = peerSeqMax ? (100.0f * peerMissed / peerSeqMax) : 0.0f;
  String out = "{";
  out += "\"node\":\"" + String(NODE_ID) + "\"";
  out += ",\"ssid\":\"" + String(WIFI_SSID) + "\"";
  out += ",\"radioReady\":" + String(radioReady ? "true" : "false");
  out += ",\"radioInitCode\":" + String(radioInitCode);
  out += ",\"rssi\":" + String(latestRssi, 1);
  out += ",\"rttMs\":" + String(latestRttMs);
  out += ",\"seqMax\":" + String(peerSeqMax);
  out += ",\"received\":" + String(peerReceived);
  out += ",\"missed\":" + String(peerMissed);
  out += ",\"duplicates\":" + String(duplicatePeer);
  out += ",\"lossPct\":" + String(peerLossPct, 2);
  out += ",\"generated\":" + String(ownGenerated);
  out += ",\"echoConfirmed\":" + String(echoConfirmed);
  out += ",\"echoMissed\":" + String(echoMissed);
  out += ",\"badEcho\":" + String(badEcho);
  out += ",\"lastSent\":\"" + lastSentMessage + "\"";
  out += ",\"lastReceived\":\"" + lastReceivedMessage + "\"";
  out += ",\"lastEcho\":\"" + lastEchoMessage + "\"";
  out += ",\"echoMatch\":" + String(lastEchoMatched ? "true" : "false");
  out += "}";
  return out;
}

static const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"><title>ESP Link Test</title>
<style>body{font-family:Arial,sans-serif;background:#111;color:#eee;margin:0;padding:18px}h1{font-size:22px;margin:0 0 6px}.small{font-size:12px;color:#aaa;margin-bottom:14px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(145px,1fr));gap:10px;margin-bottom:14px}.card{background:#1c1c1c;border:1px solid #333;border-radius:10px;padding:14px}.k{font-size:12px;color:#999}.v{font-size:24px;margin-top:6px}.msg{font-family:monospace;font-size:16px}.good{color:#49d17d}.bad{color:#ff6262}#logs{background:#050505;border:1px solid #333;border-radius:10px;padding:12px;height:38vh;overflow:auto;white-space:pre-wrap;font-family:monospace;font-size:13px}</style></head>
<body><h1>Core1262-HF Bidirectional Integrity Test</h1><div class="small" id="meta">Loading...</div>
<div class="grid"><div class="card"><div class="k">RADIO</div><div class="v" id="radio">-</div></div><div class="card"><div class="k">RSSI</div><div class="v"><span id="rssi">-</span> dBm</div></div><div class="card"><div class="k">RTT</div><div class="v"><span id="rtt">0</span> ms</div></div><div class="card"><div class="k">SEQ MAX</div><div class="v" id="seq">0</div></div><div class="card"><div class="k">RECEIVED</div><div class="v" id="received">0</div></div><div class="card"><div class="k">MISSED</div><div class="v" id="missed">0</div></div><div class="card"><div class="k">DUPLICATES</div><div class="v" id="dup">0</div></div><div class="card"><div class="k">LOSS</div><div class="v"><span id="loss">0</span>%</div></div></div>
<div class="grid"><div class="card"><div class="k">GENERATED</div><div class="v" id="generated">0</div></div><div class="card"><div class="k">ECHO CONFIRMED</div><div class="v" id="confirmed">0</div></div><div class="card"><div class="k">NO ECHO</div><div class="v" id="noecho">0</div></div><div class="card"><div class="k">BAD ECHO</div><div class="v" id="badecho">0</div></div></div>
<div class="grid"><div class="card"><div class="k">LAST SENT</div><div class="msg" id="sent">-</div></div><div class="card"><div class="k">LAST RECEIVED</div><div class="msg" id="got">-</div></div><div class="card"><div class="k">LAST ECHO</div><div class="msg" id="echo">-</div></div><div class="card"><div class="k">ECHO MATCH</div><div class="v" id="match">-</div></div></div><div id="logs">Waiting for packets...</div>
<script>async function refresh(){try{const s=await fetch('/api/status').then(r=>r.json());meta.textContent='Node '+s.node+' | WiFi '+s.ssid+' | 869 MHz GFSK | 20-byte packets';radio.textContent=s.radioReady?'OK':'ERR '+s.radioInitCode;radio.className='v '+(s.radioReady?'good':'bad');rssi.textContent=s.rssi;rtt.textContent=s.rttMs;seq.textContent=s.seqMax;received.textContent=s.received;missed.textContent=s.missed;dup.textContent=s.duplicates;loss.textContent=s.lossPct;generated.textContent=s.generated;confirmed.textContent=s.echoConfirmed;noecho.textContent=s.echoMissed;badecho.textContent=s.badEcho;sent.textContent=s.lastSent;got.textContent=s.lastReceived;echo.textContent=s.lastEcho;match.textContent=s.echoMatch?'MATCH':'MISMATCH';match.className='v '+(s.echoMatch?'good':'bad');const l=await fetch('/api/logs').then(r=>r.json());logs.textContent=l.join('\n');logs.scrollTop=logs.scrollHeight}catch(e){}}setInterval(refresh,500);refresh();</script></body></html>)HTML";

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

  nextOwnSendMs = millis() + (RANGE_ROLE_A ? 250 : NODE_B_OFFSET_MS + 250);

  Serial.println();
  Serial.println("=== Core1262-HF bidirectional integrity test ===");
  Serial.printf("Node:          ESP-%c\n", NODE_ID);
  Serial.printf("WiFi:          %s\n", WIFI_SSID);
  Serial.printf("Packet size:   %u bytes\n", (unsigned)PACKET_SIZE);
  Serial.printf("Frequency:     %.3f MHz\n", (double)RADIO_FREQ_MHZ);
  Serial.printf("TX setting:    %d dBm (requested, not measured)\n", RADIO_TX_POWER_DBM);
  Serial.printf("Bitrate:       %.1f kbps\n", (double)RADIO_BITRATE_KBPS);
  Serial.printf("RX bandwidth:  %.1f kHz\n", (double)RADIO_RX_BW_KHZ);
  if (!radioReady) addLog(String("[RADIO] init failed code=") + radioInitCode);
  else addLog(String("[RADIO] ESP-") + NODE_ID + " ready");
}

void loop() {
  server.handleClient();
  if (!radioReady) { delay(2); return; }

  processPacket();
  const uint32_t now = millis();

  if (pendingEcho && (uint32_t)(now - pendingStartedMs) >= REPLY_TIMEOUT_MS) {
    ++echoMissed;
    pendingEcho = false;
    lastEchoMatched = false;
    addLog(String("[NO ECHO] seq=") + readU24(pendingPacket + 1) + " msg=" + lastSentMessage);
  }

  if (!pendingEcho && (int32_t)(now - nextOwnSendMs) >= 0) {
    sendOwnPacket();
    nextOwnSendMs += PING_INTERVAL_MS;
    if ((int32_t)(now - nextOwnSendMs) >= 0) nextOwnSendMs = now + PING_INTERVAL_MS;
  }

  server.handleClient();
  delay(1);
}
