#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

// ESP32-WROOM -> Core1262-HF
static constexpr int PIN_SCK  = 18;
static constexpr int PIN_MISO = 19;
static constexpr int PIN_MOSI = 23;
static constexpr int PIN_NSS  = 21;
static constexpr int PIN_RST  = 14;
static constexpr int PIN_BUSY = 27;
static constexpr int PIN_DIO1 = 26;

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

static void die(const char* stage, int16_t code) {
  Serial.printf("[FATAL] %s failed: %d\n", stage, code);
  while (true) {
    delay(1000);
  }
}

static void requireOk(const char* stage, int16_t code) {
  if (code != RADIOLIB_ERR_NONE) {
    die(stage, code);
  }
}

static void printHex(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    if (data[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(data[i], HEX);
    if (i + 1 < len) {
      Serial.print(' ');
    }
  }
}

static void initRadio() {
  radioSpi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS);

  // GFSK parameters:
  // frequency MHz, bitrate kbps, frequency deviation kHz,
  // RX bandwidth kHz, TX power dBm, preamble length bits.
  int16_t state = radio.beginFSK(
      RADIO_FREQ_MHZ,
      RADIO_BITRATE_KBPS,
      RADIO_DEVIATION_KHZ,
      RADIO_RX_BW_KHZ,
      RADIO_TX_POWER_DBM,
      RADIO_PREAMBLE_BITS);

  requireOk("beginFSK", state);

  // SX1262 +22 dBm requires enough PA current headroom.
  requireOk("setCurrentLimit", radio.setCurrentLimit(140.0));

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
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  initRadio();
}

#if RANGE_ROLE_TX

static void buildPacket(uint8_t packet[PACKET_SIZE], uint32_t sequence) {
  // Bytes 0..3: little-endian sequence number.
  packet[0] = (uint8_t)(sequence);
  packet[1] = (uint8_t)(sequence >> 8);
  packet[2] = (uint8_t)(sequence >> 16);
  packet[3] = (uint8_t)(sequence >> 24);

  // Bytes 4..19: deterministic payload pattern.
  static const uint8_t payload[16] = {
    'C','O','R','E','1','2','6','2',
    '-','G','F','S','K','-','2','2'
  };
  memcpy(packet + 4, payload, sizeof(payload));
}

void loop() {
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
                  (unsigned long)txSequence,
                  state);
  }

  ++txSequence;
  delay(TX_INTERVAL_MS);
}

#else

void loop() {
  uint8_t packet[PACKET_SIZE] = {0};

  const int16_t state = radio.receive(packet, PACKET_SIZE);

  if (state == RADIOLIB_ERR_NONE) {
    const uint32_t sequence =
        ((uint32_t)packet[0]) |
        ((uint32_t)packet[1] << 8) |
        ((uint32_t)packet[2] << 16) |
        ((uint32_t)packet[3] << 24);

    Serial.printf("[RX] seq=%lu bytes=%u RSSI=%.1f dBm data=",
                  (unsigned long)sequence,
                  (unsigned)PACKET_SIZE,
                  (double)radio.getRSSI());

    printHex(packet, PACKET_SIZE);

    Serial.print(" ascii=\"");
    for (size_t i = 4; i < PACKET_SIZE; ++i) {
      const char c = (char)packet[i];
      Serial.print((c >= 32 && c <= 126) ? c : '.');
    }
    Serial.println("\"");
  } else if (state != RADIOLIB_ERR_RX_TIMEOUT) {
    Serial.printf("[RX-ERR] code=%d\n", state);
  }
}

#endif
