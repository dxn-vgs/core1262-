# Core1262-HF GFSK Range Test

Simple point-to-point range test using two ESP32-WROOM boards and two Core1262-HF (SX1262) radios.

The transmitter sends **exactly 20 bytes** once per second.  
The receiver logs every received packet to the serial console with its sequence number, RSSI and raw bytes.

## Wiring

Use the same wiring on both boards.

| Core1262-HF | ESP32-WROOM |
|---|---:|
| VCC | 3V3 |
| GND | GND |
| SCK | GPIO18 |
| MISO | GPIO19 |
| MOSI | GPIO23 |
| NSS / CS | GPIO21 |
| RESET | GPIO22 |
| BUSY | GPIO16 |
| DIO1 | GPIO17 |

All radio signals are kept on the same exposed side of the ESP32 DevKit for easier breadboard wiring.

## Radio profile

- Modulation: GFSK
- Frequency: 869.0 MHz
- TX power: +22 dBm
- Bitrate: 25 kbps
- Frequency deviation: 25 kHz
- RX bandwidth: 93.8 kHz
- Preamble: 64 bits
- Packet size: exactly 20 bytes
- TX interval: 1 second
- SX1262 current limit: 140 mA

The first 4 bytes of each packet are a little-endian sequence counter.
The remaining 16 bytes are the fixed ASCII payload:

```text
CORE1262-GFSK-22
```

## Build and flash

Install PlatformIO.

### Transmitter

```bash
pio run -e transmitter
pio run -e transmitter -t upload
pio device monitor -b 115200
```

### Receiver

```bash
pio run -e receiver
pio run -e receiver -t upload
pio device monitor -b 115200
```

## Example receiver output

```text
[RX] seq=17 bytes=20 RSSI=-82.5 dBm data=11 00 00 00 43 4F 52 45 31 32 36 32 2D 47 46 53 4B 2D 32 32 ascii="CORE1262-GFSK-22"
```

## Important for +22 dBm

Do not judge +22 dBm stability using a weak 3.3 V rail.

Use short power wiring, common ground, local decoupling close to the Core1262-HF, and a suitable 868/869 MHz antenna connected before transmitting. The firmware raises the SX1262 current limit to 140 mA to avoid the default current limit constraining high-power TX.

Also verify the frequency, power, duty-cycle and bandwidth rules that apply at the location where you perform the RF test.
