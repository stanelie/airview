# Olympus Air A01 Electronic Viewfinder

## Hardware
- Camera: Olympus Air A01 (MFT sensor, WiFi AP at 192.168.0.10)
- Display module: Waveshare ESP32-S3-Touch-LCD-1.46 (round LCD, 412x412)
- Power: Tapped directly from Olympus Air internal battery

## Display
- Controller: SPD2010 (integrates display + touch in one chip)
- Interface: QSPI
- Resolution: 412x412, 16-bit color (RGB565)
- Backlight: GPIO 5 via LEDC PWM (active high)
- Display reset: EXIO2 on TCA9554PWR I/O expander (not a direct GPIO)

## Display GPIO (QSPI)
- SCK:  GPIO 40
- D0:   GPIO 46
- D1:   GPIO 45
- D2:   GPIO 42
- D3:   GPIO 41
- CS:   GPIO 21
- TE:   GPIO 18

## Touch
- SPD2010 integrated touch (same chip as display)
- I2C bus shared with IO expander

## I2C (IO expander + touch)
- SDA: GPIO 8  (check TCA9554 driver)
- SCL: GPIO 9  (check TCA9554 driver)
- TCA9554PWR at default I2C address — EXIO2 used for display reset

## Platform
- ESP-IDF v5.3.2
- Target: esp32s3
- Flash: 16MB QIO
- PSRAM: 8MB Octal, 80MHz
- CPU: 240MHz

## Reference BSP
- ~/Documents/ESP32-S3-Touch-LCD-1.46/example/ESP-IDF-5.3.2/ESP32-S3-Touch-LCD-1.46-Test
- Waveshare factory firmware: ~/Documents/ESP32-S3-Touch-LCD-1.46/Firmware/ESP32-S3-Touch-LCD-1.46.bin

## Architecture
- Core 0: WiFi stack, HTTP client, MJPEG stream parser, JPEG decode to PSRAM
- Core 1: LVGL rendering, display push, OSD overlay, touch input

## Camera WiFi
- Camera power: physical button on camera body starts WiFi AP automatically
- SSID: AIR-A01-BHC204544
- Password: 33732272
- Camera IP: 192.168.0.10
- No Bluetooth required

## Camera API
- Base URL: http://192.168.0.10
- Live view: GET /cam.cgi?mode=liveview&value=start
- Get frame: GET /cam.cgi?mode=getliveviewframe
- Properties: GET /cam.cgi?mode=getprop&propname=...
- Shutter: GET /cam.cgi?mode=camcmd&value=snapshot

## Milestones
1. ✓ Build Waveshare LVGL demo from source — toolchain + display verified
2. Connect to Olympus Air WiFi AP, hit first cam.cgi endpoint
3. Parse MJPEG stream, extract one frame
4. Decode JPEG and display it full-screen
5. Loop into live view
6. OSD overlay (aperture, shutter, ISO)
