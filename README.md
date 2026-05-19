# airview

An electronic viewfinder (EVF) for the Olympus Air A01 lens camera, built on an ESP32-S3 with a round 412×412 display. Connect it to the camera's Wi-Fi AP and it streams a live JPEG view with a full shooting control overlay — shutter, aperture, ISO, exposure compensation, white balance, and shooting mode — all controllable by touch.

## Hardware

| Part | Details |
|------|---------|
| Board | Waveshare ESP32-S3-Touch-LCD-1.46 |
| Display | SPD2010, 412×412, round, SPI @ 80 MHz |
| SoC | ESP32-S3 (dual-core, 240 MHz, 8 MB PSRAM) |
| Flash | 16 MB QIO |

## Features

- **Live view** — streams 320×240 JPEG frames from the camera over UDP (RTP), decoded with TJpgDec and upscaled to fill the round display
- **Touch OSD** — tap any readout to cycle its value; fields dim automatically when the current shooting mode makes them irrelevant
- **Shooting modes** — cycle P / A / S / M / iA from the mode indicator
- **Exposure controls** — shutter speed, aperture, ISO, exposure compensation
- **White balance** — AWB, Sunny, Shade, Cloudy, Tungsten, Fluorescent, Underwater, Custom
- **AF box** — real-time autofocus target drawn over the live view (green = locked, red = failed)
- **Battery indicator** — live level pulled from the camera (FULL / MED / LOW / WARN / CHRG)
- **Display modes** — fill-width (letterboxed) or 1:1 centred
- **Wi-Fi setup UI** — on-device AP scanner and touch keyboard; credentials stored in NVS, never compiled in
- **Channel cache** — remembers the last connected channel so subsequent boots skip the full scan

## Building

Requires ESP-IDF v5.3.2.

```bash
. $HOME/esp/esp-idf/export.sh
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

## First-time Wi-Fi setup

On first boot (no saved credentials) the device goes directly into setup mode:

1. A list of nearby Wi-Fi networks appears on screen.
2. Tap the Olympus Air AP (typically named `AIR-A01-xxxxxxx`).
3. Enter the Wi-Fi password on the on-screen keyboard and tap **OK**.
4. The device saves the credentials to NVS and restarts.

To change credentials later, tap **Wi-Fi Setup** on the connecting screen.

## Partitions

```
nvs       0x009000  24 KB   app data (credentials, channel cache)
phy_init  0x00f000   4 KB   RF calibration
factory   0x010000   1 MB   application
```

If the device shows a `phy_init: saving new calibration data because of checksum failure` warning on every boot (adds ~2.4 s to connect time), erase the RF calibration partition once:

```bash
esptool.py --port /dev/ttyACM0 --chip esp32s3 erase_region 0xf000 0x1000
```

## Project structure

```
main/
  main.c              application (Wi-Fi, HTTP/OPC, liveview, OSD, UI)
  lcd/                SPD2010 display driver
  touch/              SPD2010 touch driver
  i2c/                I2C master helper
  exio/               TCA9554 GPIO expander
  tjpgd/              TJpgDec JPEG decoder
```

## Camera compatibility

Tested against the Olympus Air A01 using the OPC (Olympus Open Platform Camera) HTTP API. The device sets `User-Agent: OlympusCameraKit` on all requests and communicates over the camera's own Wi-Fi AP at `192.168.0.10`.
