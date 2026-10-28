SENTROVA OS - ALL FEATURES PACKAGE

Main firmware:
SENTROVA_FULL_ESP32C3_ALL_FEATURES.ino

Required companion file:
sentrova_dnn_model.h

Required Arduino libraries:
- U8g2
- TinyGPSPlus
- ESP32 BLE (included with ESP32 core)
- QRCode (install a QRCode library that provides qrcode.h)
- Preferences, WiFiClientSecure, HTTPClient (included with ESP32 core)

IMPORTANT:
1. The Telegram bot token previously pasted into chat must be revoked/regenerated in @BotFather.
2. Put the NEW token in TELEGRAM_BOT_TOKEN.
3. Put the destination Telegram chat ID in TELEGRAM_CHAT_ID.
4. Put internet Wi-Fi credentials in INTERNET_SSID / INTERNET_PASS.
5. The SENTROVA-SETUP access point remains available for the local dashboard.
6. Medical QR uses a short URL. Keep it <= 67 bytes for the version-3 QR used by the 128x64 OLED.
7. The current DNN pipeline uses reconstructed ACG = ACC(g) * 9.80665. Dataset/ESP32 ACG parity should be validated before treating the model as production-accurate.
8. HR and SpO2 are prototype estimates and are not medical-grade measurements.
9. Battery percentage is approximate and depends on the actual power path and calibration.
10. Telegram HTTPS currently uses setInsecure() for prototype convenience; production deployment should use certificate validation.

Medical QR:
Web dashboard -> Medical ID / QR -> enter URL -> GENERATE / SEND TO WATCH.
The URL is stored on the ESP32 and rendered locally as a QR on OLED page 6.
BLE can also receive: MEDICAL=<url>

OLED pages:
1 Home
2 Health
3 GPS
4 Safety
5 Settings
6 Medical QR

Telegram events:
- New SOS/fall alert
- Alert cleared/acknowledged
- Low battery
