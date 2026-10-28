#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <U8g2lib.h>
#include <TinyGPSPlus.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Preferences.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <driver/i2s.h>
#include <math.h>
#include <time.h>

// SENTROVA QRCode library
#include "sentrova_qrcode.h"
  // Install the ricmoo/QRCode Arduino library

// SENTROVA DNN model
#include "sentrova_dnn_model.h"

// ============================================================
// SENTROVA OS
// ESP32-C3 SMARTWATCH
//
// SH1106 128x64
// MPU-6500
// MAX30102
// NEO-6M
// TTP223 SOS
// TTP223 ACK
// BLE
// WiFi Dashboard
// Telegram
// Medical QR
// 60-feature DNN
//
// OLED sleeps ONLY.
// ESP32 + sensors + AI + BLE + WiFi remain active.
// ============================================================


// ============================================================
// PIN CONFIGURATION
// ============================================================

#define SDA_PIN             8
#define SCL_PIN             9

// Dedicated I2C bus for MPU-6500 (separate from OLED/MAX30102)
#define MPU_SDA_PIN         6
#define MPU_SCL_PIN         7



// NTP clock state: Indian Standard Time (UTC+05:30).
bool ntpConfigured = false;
unsigned long lastNtpStatusLog = 0;

#define SOS_PIN             4
#define ACK_PIN             5

#define GPS_RX_PIN          20
#define GPS_TX_PIN          21

#define BATTERY_ADC_PIN     3

// AUDIO (provisional pin assignment; verify against your exact ESP32-C3 board)
// INMP441 and MAX98357A share I2S BCLK/WS. INMP441 L/R must be tied to GND.
#define AUDIO_I2S_BCLK_PIN  0
#define AUDIO_I2S_WS_PIN    1
#define AUDIO_MIC_SD_PIN    10
#define AUDIO_SPK_DIN_PIN   2
#define AUDIO_SAMPLE_RATE   16000
#define AUDIO_BLOCK_FRAMES  128


// ============================================================
// I2C ADDRESSES
// ============================================================

#define MPU_ADDR            0x68
#define MAX_ADDR            0x57


// ============================================================
// OLED
// ============================================================

U8G2_SH1106_128X64_NONAME_F_HW_I2C oled(
  U8G2_R0,
  U8X8_PIN_NONE
);


// ============================================================
// GPS
// ============================================================

HardwareSerial GPSserial(1);
TinyGPSPlus gps;


// ============================================================
// PREFERENCES
// ============================================================

Preferences sentrovaPrefs;


// ============================================================
// WEB SERVER
// ============================================================

WebServer server(80);


// ============================================================
// WIFI
// ============================================================

// Local dashboard AP
const char* WIFI_SSID = "SENTROVA-SETUP";
const char* WIFI_PASS = "11111111";

IPAddress apIP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);


// Internet WiFi
const char* INTERNET_SSID = "SENTROVA";
const char* INTERNET_PASS = "44444444";


// ============================================================
// TELEGRAM
// ============================================================

// IMPORTANT:
// Generate a NEW token with @BotFather.
// Do NOT use the token previously pasted into chat.
const char* TELEGRAM_BOT_TOKEN = "8950514724:AAGslNvjP3EIoC1f5x9EBV8yT6ouVFB2L3o";

const char* TELEGRAM_CHAT_ID = "1301993851";

bool telegramEnabled = false;
bool telegramBusy = false;
bool lowBatteryAlertSent = false;


// ============================================================
// MEDICAL URL
// ============================================================

String medicalURL = "";

// Last phone notification received over BLE using command NOTIFY=<text>.
String latestNotification = "";
bool notificationUnread = false;

static const size_t MEDICAL_URL_MAX = 67;

#define MEDICAL_QR_VERSION 4
#define MEDICAL_QR_BUFFER_SIZE 220

QRCode medicalQR;
uint8_t medicalQRData[MEDICAL_QR_BUFFER_SIZE];


// ============================================================
// SENSOR VARIABLES
// ============================================================

float accX = 0.0f;
float accY = 0.0f;
float accZ = 0.0f;

float gyroX = 0.0f;
float gyroY = 0.0f;
float gyroZ = 0.0f;

float accMagnitude = 0.0f;
float gyroMagnitude = 0.0f;

uint32_t irValue = 0;
uint32_t redValue = 0;

bool fingerDetected = false;

bool mpuOK = false;
bool maxOK = false;

bool gpsFix = false;
int gpsSatellites = 0;

double gpsLat = 0.0;
double gpsLng = 0.0;


// ============================================================
// SAFETY
// ============================================================

bool alertActive = false;
String alertReason = "SYSTEM SECURE";

unsigned long lastFallAlert = 0;

const unsigned long FALL_COOLDOWN = 10000UL;


// ============================================================
// TOUCH
// ============================================================

bool sosTouched = false;
bool ackTouched = false;

unsigned long sosStart = 0;
unsigned long ackStart = 0;

bool sosHandled = false;
bool ackHandled = false;


// ============================================================
// OLED POWER
// ============================================================

bool oledAwake = true;

const unsigned long OLED_TIMEOUT_MS = 30000UL;

unsigned long lastUserActivity = 0;


// ============================================================
// WATCH UI
// ============================================================

int watchPage = 0;

const int TOTAL_PAGES = 9;

unsigned long lastDisplayUpdate = 0;
unsigned long bootTime = 0;


// ============================================================
// BATTERY
// ============================================================

const float BAT_R_TOP = 100000.0f;
const float BAT_R_BOTTOM = 100000.0f;

const float BAT_ADC_REF = 3.30f;
const float BAT_ADC_MAX = 4095.0f;

float batteryVoltage = 0.0f;
int batteryPercent = 0;
bool batteryLow = false;


// ============================================================
// 60-FEATURE DNN
// ============================================================

static const int FEATURE_WINDOW = 50;
static const int FEATURE_STEP = 25;

// ACC
float fAccX[FEATURE_WINDOW];
float fAccY[FEATURE_WINDOW];
float fAccZ[FEATURE_WINDOW];
float fAccMag[FEATURE_WINDOW];

// ACG
float fAcgX[FEATURE_WINDOW];
float fAcgY[FEATURE_WINDOW];
float fAcgZ[FEATURE_WINDOW];
float fAcgMag[FEATURE_WINDOW];

// GYRO
float fGyroX[FEATURE_WINDOW];
float fGyroY[FEATURE_WINDOW];
float fGyroZ[FEATURE_WINDOW];
float fGyroMag[FEATURE_WINDOW];

int featureSamples = 0;
int featureSamplesSinceInference = 0;

uint32_t featureWindowNumber = 0;

float featureVector[60] = {0};

float aiFallProbability = 0.0f;
bool aiFall = false;


// ============================================================
// MAX30102
// ============================================================

static const int PPG_WINDOW = 100;

uint32_t ppgIR[PPG_WINDOW];
uint32_t ppgRED[PPG_WINDOW];

int ppgIndex = 0;
int ppgCount = 0;

float heartRate = 0.0f;
float spo2Value = 0.0f;

bool heartRateValid = false;
bool spo2Valid = false;

unsigned long lastBeatMs = 0;

float hrAccumulator = 0.0f;
int hrAccumulatorCount = 0;


// ============================================================
// BLE
// ============================================================

#define SENTROVA_BLE_SERVICE \
  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"

#define SENTROVA_BLE_RX \
  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

#define SENTROVA_BLE_TX \
  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer* sentrovaBLEServer = nullptr;
BLECharacteristic* sentrovaBLERX = nullptr;
BLECharacteristic* sentrovaBLETX = nullptr;

bool bleConnected = false;

// Audio status. The ESP32-C3 firmware can generate local SOS tones and read
// INMP441 samples; standard Bluetooth A2DP speaker streaming is NOT supported
// by ESP32-C3. BLE is used for control/telemetry only.
bool audioHardwareOK = false;
bool speakerAlertsEnabled = true;
float microphoneLevel = 0.0f;
unsigned long speakerTestUntil = 0;
float audioTonePhase = 0.0f;


// ============================================================
// FORWARD DECLARATIONS
// ============================================================

bool telegramConfigValid();

void wakeOLED();
void sleepOLED();
void handleOLEDTimeout();

void activateAlert(const String& reason);
void clearAlert();

bool setMedicalURL(const String& url);

void sendTelegramMessage(const String& message);

void updateGPS();

bool readMPU();
bool initializeMPU();

bool initializeMAX30102();
void updateMAXHealthBuffer();
void updateHealthMetrics();

void processFeatureSample();
void build60Features();
void print60Features();
void runAIInference();
void updateAIFallAlert();

void readBattery();

void initSENTROVABLE();
void sendBLETElemetry();
bool initSENTROVAAudio();
void serviceSENTROVAAudio();
void requestSpeakerTest();

void startWiFi();
void maintainInternetWiFi();
void maintainNtpTime();

void startWebServer();

String dashboard();


// ============================================================
// BIT-BANG I2C PRIMITIVES (software I2C on GPIO 6/7)
// ============================================================
//
// ESP32-C3 has ONLY ONE hardware I2C controller (Wire/0).
// Using TwoWire(1) fails with error 4 (ESP_FAIL) on every address.
// Solution: bit-bang GPIO 6/7 manually.

static inline void bbSdaRelease() { pinMode(MPU_SDA_PIN, INPUT_PULLUP); }
static inline void bbSdaLow()     { pinMode(MPU_SDA_PIN, OUTPUT); digitalWrite(MPU_SDA_PIN, LOW); }
static inline void bbSclRelease() { pinMode(MPU_SCL_PIN, INPUT_PULLUP); }
static inline void bbSclLow()     { pinMode(MPU_SCL_PIN, OUTPUT); digitalWrite(MPU_SCL_PIN, LOW); }
static inline int  bbSdaRead()    { pinMode(MPU_SDA_PIN, INPUT_PULLUP); return digitalRead(MPU_SDA_PIN); }
static inline void bbDelay()      { delayMicroseconds(5); }


static void bbI2cStart()
{
  bbSdaRelease();
  bbSclRelease();
  bbDelay();
  bbSdaLow();
  bbDelay();
  bbSclLow();
  bbDelay();
}


static void bbI2cStop()
{
  bbSdaLow();
  bbDelay();
  bbSclRelease();
  bbDelay();
  bbSdaRelease();
  bbDelay();
}


static bool bbI2cWriteByte(uint8_t b)
{
  for (int i = 7; i >= 0; i--) {
    if (b & (1 << i)) bbSdaRelease();
    else              bbSdaLow();
    bbDelay();
    bbSclRelease();
    bbDelay();
    bbSclLow();
    bbDelay();
  }
  bbSdaRelease();
  bbDelay();
  bbSclRelease();
  bbDelay();
  int ack = bbSdaRead();
  bbSclLow();
  bbDelay();
  return (ack == 0);
}


static uint8_t bbI2cReadByte(bool sendAck)
{
  uint8_t b = 0;
  bbSdaRelease();
  for (int i = 7; i >= 0; i--) {
    bbDelay();
    bbSclRelease();
    bbDelay();
    if (bbSdaRead()) b |= (1 << i);
    bbSclLow();
    bbDelay();
  }
  if (sendAck) bbSdaLow();
  else         bbSdaRelease();
  bbDelay();
  bbSclRelease();
  bbDelay();
  bbSclLow();
  bbSdaRelease();
  bbDelay();
  return b;
}


// ============================================================
// PREFERENCES
// ============================================================

 
// PREFERENCES
// ============================================================

void loadPersistentSettings()
{
  if (!sentrovaPrefs.begin("sentrova", false))
  {
    Serial.println("Preferences: FAILED");
    return;
  }

  medicalURL =
    sentrovaPrefs.getString(
      "medical_url",
      ""
    );

  Serial.println();
  Serial.println("================================");
  Serial.println("PERSISTENT SETTINGS");
  Serial.println("================================");

  if (medicalURL.length() > 0)
  {
    Serial.print("Medical URL: ");
    Serial.println(medicalURL);
  }
  else
  {
    Serial.println("Medical URL: NONE");
  }

  Serial.println("================================");
}


// ============================================================
// I2C HELPERS
// ============================================================

void writeRegister(
  uint8_t address,
  uint8_t reg,
  uint8_t value
)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}


uint8_t readRegister(
  uint8_t address,
  uint8_t reg
)
{
  Wire.beginTransmission(address);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
    return 0xFF;

  Wire.requestFrom(
    address,
    (uint8_t)1
  );

  if (Wire.available())
    return Wire.read();

  return 0xFF;
}


bool readRegisters(
  uint8_t address,
  uint8_t reg,
  uint8_t* buffer,
  uint8_t length
)
{
  Wire.beginTransmission(address);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0)
    return false;

  uint8_t received =
    Wire.requestFrom(
      address,
      length
    );

  if (received != length)
    return false;

  for (uint8_t i = 0; i < length; i++)
    buffer[i] = Wire.read();

  return true;
}


// Dedicated MPU-6500 I2C helpers. OLED and MAX30102 continue using Wire.
// ============================================================
// MPU REGISTER ACCESS (bit-bang, same signatures as before)
// ============================================================

bool writeMPURegister(uint8_t address, uint8_t reg, uint8_t value)
{
  bbI2cStart();
  bool ok = bbI2cWriteByte(address << 1);
  if (ok) ok = bbI2cWriteByte(reg);
  if (ok) ok = bbI2cWriteByte(value);
  bbI2cStop();
  return ok;
}

uint8_t readMPURegister(uint8_t address, uint8_t reg)
{
  bbI2cStart();
  if (!bbI2cWriteByte(address << 1)) { bbI2cStop(); return 0xFF; }
  if (!bbI2cWriteByte(reg))          { bbI2cStop(); return 0xFF; }
  bbI2cStart();
  if (!bbI2cWriteByte((address << 1) | 1)) { bbI2cStop(); return 0xFF; }
  uint8_t b = bbI2cReadByte(false);
  bbI2cStop();
  return b;
}

bool readMPURegisters(uint8_t address, uint8_t reg, uint8_t* buffer, uint8_t length)
{
  bbI2cStart();
  if (!bbI2cWriteByte(address << 1)) { bbI2cStop(); return false; }
  if (!bbI2cWriteByte(reg))          { bbI2cStop(); return false; }
  bbI2cStart();
  if (!bbI2cWriteByte((address << 1) | 1)) { bbI2cStop(); return false; }

  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = bbI2cReadByte(i < (length - 1));
  }
  bbI2cStop();
  return true;
}

// ============================================================
// MPU-6500
// ============================================================

#define MPU_PWR_MGMT_1   0x6B
#define MPU_WHO_AM_I     0x75
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_GYRO_CONFIG  0x1B
#define MPU_ACCEL_CONFIG 0x1C


bool initializeMPU()
{
  // Bit-bang probe: write address, expect ACK.
  bool found = false;
  for (uint8_t attempt = 1; attempt <= 3; ++attempt)
  {
    bbI2cStart();
    bool ack = bbI2cWriteByte(MPU_ADDR << 1);
    bbI2cStop();

    if (ack)
    {
      found = true;
      break;
    }
    Serial.printf("MPU bit-bang probe attempt %u failed\n", attempt);
    delay(25);
  }

  if (!found)
  {
    Serial.println("MPU: no ACK on bit-bang I2C (GPIO6/7). Check wiring and pull-ups.");
    return false;
  }
  uint8_t who = 0xFF;
  bool identityRead = false;
  for (uint8_t attempt = 1; attempt <= 3; ++attempt)
  {
    who = readMPURegister(MPU_ADDR, MPU_WHO_AM_I);
    if (who != 0xFF)
    {
      identityRead = true;
      break;
    }
    delay(20);
  }

  Serial.print("MPU WHO_AM_I: 0x");
  Serial.println(who, HEX);

  if (!identityRead)
  {
    Serial.println("MPU: I2C ACK works, but WHO_AM_I read failed.");
    return false;
  }

  // The user's sensor returned 0x70 in the standalone register test.
  // Accept known MPU-6500/6050-family IDs, but log the observed identity.
  if (who != 0x70 && who != 0x71 && who != 0x68 && who != 0x69)
  {
    Serial.println("MPU: unrecognized WHO_AM_I; refusing to enable motion/DNN pipeline.");
    return false;
  }

  // Wake the device and configure the same ranges used by readMPU():
  // ±8 g => 4096 LSB/g; ±500 dps => 65.5 LSB/(degree/sec).
  if (!writeMPURegister(MPU_ADDR, MPU_PWR_MGMT_1, 0x00))
  {
    Serial.println("MPU: wake-up write failed.");
    return false;
  }
  delay(100);

  if (!writeMPURegister(MPU_ADDR, MPU_ACCEL_CONFIG, 0x10))
  {
    Serial.println("MPU: accelerometer configuration write failed.");
    return false;
  }

  if (!writeMPURegister(MPU_ADDR, MPU_GYRO_CONFIG, 0x08))
  {
    Serial.println("MPU: gyroscope configuration write failed.");
    return false;
  }

  // Verify that configuration registers can be read back.
  uint8_t power = readMPURegister(MPU_ADDR, MPU_PWR_MGMT_1);
  uint8_t accelConfig = readMPURegister(MPU_ADDR, MPU_ACCEL_CONFIG);
  uint8_t gyroConfig = readMPURegister(MPU_ADDR, MPU_GYRO_CONFIG);

  Serial.printf("MPU config readback: PWR=0x%02X ACCEL=0x%02X GYRO=0x%02X\n",
                power, accelConfig, gyroConfig);

  if (power == 0xFF || accelConfig == 0xFF || gyroConfig == 0xFF)
  {
    Serial.println("MPU: configuration readback failed.");
    return false;
  }

  if ((power & 0x40) != 0 || (accelConfig & 0x18) != 0x10 || (gyroConfig & 0x18) != 0x08)
  {
    Serial.println("MPU: configuration did not match requested ranges.");
    return false;
  }

  uint8_t sample[14];
  if (!readMPURegisters(MPU_ADDR, MPU_ACCEL_XOUT_H, sample, sizeof(sample)))
  {
    Serial.println("MPU: initial motion-register read failed.");
    return false;
  }

  Serial.println("MPU: identity, configuration and data-register checks passed.");
  return true;
}


bool readMPU()
{
  uint8_t data[14];

  if (
    !readMPURegisters(
      MPU_ADDR,
      MPU_ACCEL_XOUT_H,
      data,
      14
    )
  )
  {
    return false;
  }

  int16_t rawAx =
    ((int16_t)data[0] << 8) |
    data[1];

  int16_t rawAy =
    ((int16_t)data[2] << 8) |
    data[3];

  int16_t rawAz =
    ((int16_t)data[4] << 8) |
    data[5];

  int16_t rawGx =
    ((int16_t)data[8] << 8) |
    data[9];

  int16_t rawGy =
    ((int16_t)data[10] << 8) |
    data[11];

  int16_t rawGz =
    ((int16_t)data[12] << 8) |
    data[13];

  accX = rawAx / 4096.0f;
  accY = rawAy / 4096.0f;
  accZ = rawAz / 4096.0f;

  gyroX =
    (rawGx / 65.5f) *
    DEG_TO_RAD;

  gyroY =
    (rawGy / 65.5f) *
    DEG_TO_RAD;

  gyroZ =
    (rawGz / 65.5f) *
    DEG_TO_RAD;

  accMagnitude =
    sqrt(
      accX * accX +
      accY * accY +
      accZ * accZ
    );

  gyroMagnitude =
    sqrt(
      gyroX * gyroX +
      gyroY * gyroY +
      gyroZ * gyroZ
    );

  return true;
}


// ============================================================
// MAX30102
// ============================================================

#define MAX_FIFO_DATA   0x07
#define MAX_MODE_CONFIG 0x09
#define MAX_SPO2_CONFIG 0x0A
#define MAX_LED1_PA     0x0C
#define MAX_LED2_PA     0x0D
#define MAX_PART_ID     0xFF


bool initializeMAX30102()
{
  uint8_t partID =
    readRegister(
      MAX_ADDR,
      MAX_PART_ID
    );

  Serial.print("MAX30102 PART ID: 0x");
  Serial.println(partID, HEX);

  if (partID != 0x15)
    return false;

  writeRegister(
    MAX_ADDR,
    MAX_MODE_CONFIG,
    0x40
  );

  delay(100);

  writeRegister(MAX_ADDR, 0x08, 0x5F);
  writeRegister(MAX_ADDR, 0x04, 0x00);
  writeRegister(MAX_ADDR, 0x05, 0x00);
  writeRegister(MAX_ADDR, 0x06, 0x00);

  writeRegister(
    MAX_ADDR,
    MAX_MODE_CONFIG,
    0x03
  );

  writeRegister(
    MAX_ADDR,
    MAX_SPO2_CONFIG,
    0x27
  );

  writeRegister(
    MAX_ADDR,
    MAX_LED1_PA,
    0x24
  );

  writeRegister(
    MAX_ADDR,
    MAX_LED2_PA,
    0x24
  );

  return true;
}


// ============================================================
// GPS
// ============================================================

void updateGPS()
{
  while (GPSserial.available())
  {
    gps.encode(
      GPSserial.read()
    );
  }

  gpsFix =
    gps.location.isValid();

  if (gpsFix)
  {
    gpsLat =
      gps.location.lat();

    gpsLng =
      gps.location.lng();
  }

  if (gps.satellites.isValid())
  {
    gpsSatellites =
      gps.satellites.value();
  }
}


// ============================================================
// MEDICAL URL
// ============================================================

bool setMedicalURL(
  const String& url
)
{
  String cleanURL = url;
  cleanURL.trim();

  if (cleanURL.length() == 0)
  {
    Serial.println("Medical URL: EMPTY");
    return false;
  }

  if (
    !cleanURL.startsWith("http://") &&
    !cleanURL.startsWith("https://")
  )
  {
    Serial.println("Medical URL: INVALID");
    return false;
  }

  if (
    cleanURL.length() >
    MEDICAL_URL_MAX
  )
  {
    Serial.println("Medical URL: TOO LONG");
    return false;
  }

  medicalURL = cleanURL;

  sentrovaPrefs.putString(
    "medical_url",
    medicalURL
  );

  Serial.println();
  Serial.println("======================================");
  Serial.println("       MEDICAL URL UPDATED");
  Serial.println("======================================");
  Serial.println(medicalURL);
  Serial.println("======================================");

  wakeOLED();

  return true;
}


// ============================================================
// QR DISPLAY
// ============================================================

void drawMedicalQR()
{
  oled.clearBuffer();

  if (!medicalURL.length())
  {
    oled.setFont(u8g2_font_6x13B_tf);

    oled.drawStr(
      17,
      22,
      "MEDICAL QR"
    );

    oled.setFont(u8g2_font_5x8_tf);

    oled.drawStr(
      18,
      38,
      "Set URL in"
    );

    oled.drawStr(
      13,
      49,
      "Web Dashboard"
    );

    oled.drawStr(
      31,
      61,
      "NO URL"
    );

    oled.sendBuffer();

    return;
  }

  if (
    medicalURL.length() >
    MEDICAL_URL_MAX
  )
  {
    oled.setFont(u8g2_font_6x13B_tf);

    oled.drawStr(
      13,
      25,
      "URL TOO LONG"
    );

    oled.setFont(u8g2_font_5x8_tf);

    oled.drawStr(
      9,
      40,
      "Use shorter URL"
    );

    oled.drawStr(
      21,
      52,
      "for QR display"
    );

    oled.sendBuffer();

    return;
  }

  // ricmoo/QRCode API initializes the QRCode in-place (void return).
  qrcode_initText(
    &medicalQR,
    medicalQRData,
    MEDICAL_QR_VERSION,
    ECC_LOW,
    medicalURL.c_str()
  );

  const int size =
    medicalQR.size;

  const int scale = 1;
  const int quietZone = 2;

  const int qrPixels =
    (size + quietZone * 2) *
    scale;

  const int x0 =
    (128 - qrPixels) / 2 +
    quietZone;

  const int y0 =
    (64 - qrPixels) / 2 +
    quietZone;

  for (int y = 0; y < size; y++)
  {
    for (int x = 0; x < size; x++)
    {
      if (
        qrcode_getModule(
          &medicalQR,
          x,
          y
        )
      )
      {
        oled.drawPixel(
          x0 + x,
          y0 + y
        );
      }
    }
  }

  oled.sendBuffer();
}


// ============================================================
// BATTERY
// ============================================================

void readBattery()
{
  uint32_t total = 0;

  for (int i = 0; i < 8; i++)
    total += analogRead(
      BATTERY_ADC_PIN
    );

  float adc =
    total / 8.0f;

  float vAdc =
    (adc / BAT_ADC_MAX) *
    BAT_ADC_REF;

  batteryVoltage =
    vAdc *
    (
      (BAT_R_TOP + BAT_R_BOTTOM) /
      BAT_R_BOTTOM
    );

  if (batteryVoltage >= 4.20f)
    batteryPercent = 100;

  else if (batteryVoltage >= 4.10f)
    batteryPercent =
      90 +
      (int)(
        (batteryVoltage - 4.10f) *
        100
      );

  else if (batteryVoltage >= 4.00f)
    batteryPercent =
      80 +
      (int)(
        (batteryVoltage - 4.00f) *
        100
      );

  else if (batteryVoltage >= 3.90f)
    batteryPercent =
      65 +
      (int)(
        (batteryVoltage - 3.90f) *
        150
      );

  else if (batteryVoltage >= 3.80f)
    batteryPercent =
      50 +
      (int)(
        (batteryVoltage - 3.80f) *
        150
      );

  else if (batteryVoltage >= 3.70f)
    batteryPercent =
      35 +
      (int)(
        (batteryVoltage - 3.70f) *
        150
      );

  else if (batteryVoltage >= 3.60f)
    batteryPercent =
      20 +
      (int)(
        (batteryVoltage - 3.60f) *
        150
      );

  else if (batteryVoltage >= 3.50f)
    batteryPercent =
      10 +
      (int)(
        (batteryVoltage - 3.50f) *
        100
      );

  else if (batteryVoltage >= 3.30f)
    batteryPercent =
      3 +
      (int)(
        (batteryVoltage - 3.30f) *
        35
      );

  else
    batteryPercent = 0;

  batteryPercent =
    constrain(
      batteryPercent,
      0,
      100
    );

  batteryLow =
    batteryVoltage < 3.50f;
}


// ============================================================
// MAX30102 HEALTH
// ============================================================

void updateHealthMetrics()
{
  if (ppgCount < 25)
    return;

  int n = ppgCount;

  uint64_t sum = 0;

  uint32_t minIR = 0xFFFFFFFF;
  uint32_t maxIR = 0;

  for (int i = 0; i < n; i++)
  {
    uint32_t v =
      ppgIR[i];

    sum += v;

    if (v < minIR)
      minIR = v;

    if (v > maxIR)
      maxIR = v;
  }

  float mean =
    (float)sum / n;

  float amp =
    (float)maxIR -
    (float)minIR;

  if (amp < 500)
    return;

  float threshold =
    mean +
    amp * 0.15f;

  int start =
    (ppgCount < PPG_WINDOW)
    ? 0
    : ppgIndex;

  static bool wasAbove = false;

  for (int k = 0; k < n; k++)
  {
    int idx =
      (start + k) %
      PPG_WINDOW;

    bool above =
      ppgIR[idx] >
      threshold;

    if (
      above &&
      !wasAbove
    )
    {
      unsigned long now =
        millis();

      if (lastBeatMs)
      {
        unsigned long dt =
          now - lastBeatMs;

        if (
          dt >= 300 &&
          dt <= 2000
        )
        {
          float bpm =
            60000.0f /
            dt;

          if (
            bpm >= 30 &&
            bpm <= 220
          )
          {
            hrAccumulator += bpm;
            hrAccumulatorCount++;

            if (
              hrAccumulatorCount >= 4
            )
            {
              heartRate =
                hrAccumulator /
                hrAccumulatorCount;

              hrAccumulator = 0;
              hrAccumulatorCount = 0;
            }

            heartRateValid = true;
          }
        }
      }

      lastBeatMs = now;
    }

    wasAbove = above;
  }

  double irDC = 0;
  double redDC = 0;

  double irAC = 0;
  double redAC = 0;

  for (int i = 0; i < n; i++)
  {
    irDC += ppgIR[i];
    redDC += ppgRED[i];
  }

  irDC /= n;
  redDC /= n;

  for (int i = 0; i < n; i++)
  {
    double di =
      ppgIR[i] - irDC;

    double dr =
      ppgRED[i] - redDC;

    irAC += di * di;
    redAC += dr * dr;
  }

  irAC =
    sqrt(irAC / n);

  redAC =
    sqrt(redAC / n);

  if (
    irDC > 0 &&
    redDC > 0 &&
    irAC > 1 &&
    redAC > 1
  )
  {
    double ratio =
      (redAC / redDC) /
      (irAC / irDC);

    float s =
      110.0f -
      25.0f * ratio;

    if (
      s > 70 &&
      s <= 100
    )
    {
      spo2Value = s;
      spo2Valid = true;
    }
  }
}


void updateMAXHealthBuffer()
{
  if (!maxOK)
    return;

  uint8_t data[6];

  if (
    !readRegisters(
      MAX_ADDR,
      MAX_FIFO_DATA,
      data,
      6
    )
  )
    return;

  redValue =
    ((uint32_t)(data[0] & 0x03) << 16) |
    ((uint32_t)data[1] << 8) |
    data[2];

  irValue =
    ((uint32_t)(data[3] & 0x03) << 16) |
    ((uint32_t)data[4] << 8) |
    data[5];

  fingerDetected =
    irValue > 5000;

  if (fingerDetected)
  {
    ppgIR[ppgIndex] =
      irValue;

    ppgRED[ppgIndex] =
      redValue;

    ppgIndex =
      (ppgIndex + 1) %
      PPG_WINDOW;

    if (ppgCount < PPG_WINDOW)
      ppgCount++;

    updateHealthMetrics();
  }
  else
  {
    heartRateValid = false;
    spo2Valid = false;

    heartRate = 0;
    spo2Value = 0;
  }
}


// ============================================================
// 60-FEATURE ENGINE
// ============================================================

float sentrovaMedian(
  float* input,
  int n
)
{
  static float temp[FEATURE_WINDOW];

  for (int i = 0; i < n; i++)
    temp[i] = input[i];

  for (int i = 1; i < n; i++)
  {
    float key = temp[i];

    int j = i - 1;

    while (
      j >= 0 &&
      temp[j] > key
    )
    {
      temp[j + 1] =
        temp[j];

      j--;
    }

    temp[j + 1] =
      key;
  }

  if (n % 2)
    return temp[n / 2];

  return (
    temp[n / 2 - 1] +
    temp[n / 2]
  ) * 0.5f;
}


void sentrovaStats(
  float* a,
  int n,
  double& mean,
  double& sd,
  double& mn,
  double& mx,
  double& med
)
{
  mean = 0;
  mn = a[0];
  mx = a[0];

  for (int i = 0; i < n; i++)
  {
    mean += a[i];

    if (a[i] < mn)
      mn = a[i];

    if (a[i] > mx)
      mx = a[i];
  }

  mean /= n;

  double ss = 0;

  for (int i = 0; i < n; i++)
  {
    double d =
      a[i] - mean;

    ss += d * d;
  }

  sd =
    (n > 1)
    ? sqrt(ss / (n - 1))
    : 0.0;

  med =
    sentrovaMedian(
      a,
      n
    );
}


void sentrovaFillSensor(
  float* x,
  float* y,
  float* z,
  float* mag,
  int base
)
{
  double m;
  double sd;
  double mn;
  double mx;
  double med;

  sentrovaStats(
    x,
    FEATURE_WINDOW,
    m,
    sd,
    mn,
    mx,
    med
  );

  featureVector[base + 0] = m;
  featureVector[base + 1] = sd;
  featureVector[base + 2] = mn;
  featureVector[base + 3] = mx;
  featureVector[base + 4] = med;

  sentrovaStats(
    y,
    FEATURE_WINDOW,
    m,
    sd,
    mn,
    mx,
    med
  );

  featureVector[base + 5] = m;
  featureVector[base + 6] = sd;
  featureVector[base + 7] = mn;
  featureVector[base + 8] = mx;
  featureVector[base + 9] = med;

  sentrovaStats(
    z,
    FEATURE_WINDOW,
    m,
    sd,
    mn,
    mx,
    med
  );

  featureVector[base + 10] = m;
  featureVector[base + 11] = sd;
  featureVector[base + 12] = mn;
  featureVector[base + 13] = mx;
  featureVector[base + 14] = med;

  sentrovaStats(
    mag,
    FEATURE_WINDOW,
    m,
    sd,
    mn,
    mx,
    med
  );

  featureVector[base + 15] = m;
  featureVector[base + 16] = sd;
  featureVector[base + 17] = mn;
  featureVector[base + 18] = mx;
  featureVector[base + 19] = med;
}


void build60Features()
{
  sentrovaFillSensor(
    fAccX,
    fAccY,
    fAccZ,
    fAccMag,
    0
  );

  sentrovaFillSensor(
    fAcgX,
    fAcgY,
    fAcgZ,
    fAcgMag,
    20
  );

  sentrovaFillSensor(
    fGyroX,
    fGyroY,
    fGyroZ,
    fGyroMag,
    40
  );
}


void print60Features()
{
  Serial.println();
  Serial.println(
    "========================================"
  );

  Serial.printf(
    "60-FEATURE WINDOW #%lu\n",
    (unsigned long)
    featureWindowNumber
  );

  Serial.println(
    "========================================"
  );

  for (int i = 0; i < 60; i++)
  {
    Serial.printf(
      "%02d %-18s = %.8f\n",
      i + 1,
      SENTROVA_DNN_FEATURE_NAMES[i],
      featureVector[i]
    );
  }

  Serial.println(
    "========================================"
  );
}


void runAIInference()
{
  aiFallProbability =
    sentrovaDnnPredictFallProbability(
      featureVector
    );

  aiFall =
    (
      aiFallProbability >=
      SENTROVA_DNN_THRESHOLD
    );

  Serial.printf(
    "DNN: %s | FALL PROBABILITY: %.2f%% | THRESHOLD: %.2f\n",
    aiFall ? "FALL" : "ADL",
    aiFallProbability * 100.0f,
    SENTROVA_DNN_THRESHOLD
  );
}


void updateAIFallAlert()
{
  if (!aiFall)
    return;

  if (
    millis() -
    lastFallAlert <
    FALL_COOLDOWN
  )
    return;

  activateAlert(
    "AI FALL DETECTED"
  );
}


void processFeatureSample()
{
  if (!mpuOK)
    return;

  if (
    featureSamples <
    FEATURE_WINDOW
  )
  {
    int i =
      featureSamples++;

    fAccX[i] =
      accX;

    fAccY[i] =
      accY;

    fAccZ[i] =
      accZ;

    fAccMag[i] =
      accMagnitude;

    // Current model pipeline reconstructs ACG.
    fAcgX[i] =
      accX * 9.80665f;

    fAcgY[i] =
      accY * 9.80665f;

    fAcgZ[i] =
      accZ * 9.80665f;

    fAcgMag[i] =
      accMagnitude * 9.80665f;

    fGyroX[i] =
      gyroX;

    fGyroY[i] =
      gyroY;

    fGyroZ[i] =
      gyroZ;

    fGyroMag[i] =
      gyroMagnitude;
  }

  if (
    featureSamples >=
    FEATURE_WINDOW &&
    featureSamplesSinceInference == 0
  )
  {
    featureWindowNumber++;

    build60Features();

    print60Features();

    runAIInference();

    updateAIFallAlert();
  }

  if (
    featureSamples >=
    FEATURE_WINDOW
  )
  {
    featureSamplesSinceInference++;

    if (
      featureSamplesSinceInference >=
      FEATURE_STEP
    )
    {
      for (int i = 0; i < 25; i++)
      {
        fAccX[i] =
          fAccX[i + 25];

        fAccY[i] =
          fAccY[i + 25];

        fAccZ[i] =
          fAccZ[i + 25];

        fAccMag[i] =
          fAccMag[i + 25];

        fAcgX[i] =
          fAcgX[i + 25];

        fAcgY[i] =
          fAcgY[i + 25];

        fAcgZ[i] =
          fAcgZ[i + 25];

        fAcgMag[i] =
          fAcgMag[i + 25];

        fGyroX[i] =
          fGyroX[i + 25];

        fGyroY[i] =
          fGyroY[i + 25];

        fGyroZ[i] =
          fGyroZ[i + 25];

        fGyroMag[i] =
          fGyroMag[i + 25];
      }

      featureSamples = 25;
      featureSamplesSinceInference = 0;
    }
  }
}


// ============================================================
// OLED POWER
// ============================================================

void wakeOLED()
{
  lastUserActivity =
    millis();

  if (!oledAwake)
  {
    oled.setPowerSave(0);
    oledAwake = true;
  }
}


void sleepOLED()
{
  if (oledAwake)
  {
    oled.setPowerSave(1);
    oledAwake = false;
  }
}


void handleOLEDTimeout()
{
  if (
    oledAwake &&
    !alertActive &&
    millis() -
    lastUserActivity >=
    OLED_TIMEOUT_MS
  )
  {
    sleepOLED();
  }
}


// ============================================================
// TELEGRAM CONFIG
// ============================================================

bool telegramConfigValid()
{
  String token =
    String(TELEGRAM_BOT_TOKEN);

  String chat =
    String(TELEGRAM_CHAT_ID);

  if (token.length() == 0)
    return false;

  if (chat.length() == 0)
    return false;

  if (
    token ==
    "PASTE_NEW_BOT_TOKEN_HERE"
  )
    return false;

  if (
    chat ==
    "PASTE_CHAT_ID_HERE"
  )
    return false;

  return true;
}


// ============================================================
// TELEGRAM MESSAGE
// ============================================================

void sendTelegramMessage(
  const String& message
)
{
  if (telegramBusy)
  {
    Serial.println(
      "Telegram: BUSY"
    );

    return;
  }

  if (!telegramConfigValid())
  {
    Serial.println(
      "Telegram: configuration not set"
    );

    return;
  }

  if (
    WiFi.status() !=
    WL_CONNECTED
  )
  {
    Serial.println(
      "Telegram: Internet WiFi NOT connected"
    );

    return;
  }

  telegramBusy = true;

  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "       TELEGRAM CONNECTION TEST"
  );

  Serial.println(
    "======================================"
  );

  Serial.print(
    "STA IP      : "
  );

  Serial.println(
    WiFi.localIP()
  );

  Serial.print(
    "Gateway     : "
  );

  Serial.println(
    WiFi.gatewayIP()
  );

  Serial.print(
    "DNS         : "
  );

  Serial.println(
    WiFi.dnsIP()
  );

  Serial.print(
    "RSSI        : "
  );

  Serial.print(
    WiFi.RSSI()
  );

  Serial.println(
    " dBm"
  );


  // ----------------------------------------------------------
  // DNS
  // ----------------------------------------------------------

  IPAddress telegramIP;

  Serial.println(
    "DNS: resolving api.telegram.org..."
  );

  if (
    !WiFi.hostByName(
      "api.telegram.org",
      telegramIP
    )
  )
  {
    Serial.println(
      "DNS ERROR: api.telegram.org could not be resolved"
    );

    telegramBusy = false;

    return;
  }

  Serial.print(
    "Telegram IP : "
  );

  Serial.println(
    telegramIP
  );


  // ----------------------------------------------------------
  // TCP 443 TEST
  // ----------------------------------------------------------

  WiFiClientSecure testClient;

  testClient.setInsecure();
  testClient.setTimeout(8000);

  Serial.println(
    "TCP: connecting to api.telegram.org:443..."
  );

  unsigned long tcpStart =
    millis();

  if (
    !testClient.connect(
      telegramIP,
      443
    )
  )
  {
    Serial.print(
      "TCP ERROR after "
    );

    Serial.print(
      millis() -
      tcpStart
    );

    Serial.println(
      " ms"
    );

    testClient.stop();

    Serial.println(
      "Telegram TCP connection failed."
    );

    Serial.println(
      "Try a mobile hotspot to isolate the ISP/network."
    );

    telegramBusy = false;

    return;
  }

  Serial.print(
    "TCP: CONNECTED in "
  );

  Serial.print(
    millis() -
    tcpStart
  );

  Serial.println(
    " ms"
  );

  testClient.stop();


  // ----------------------------------------------------------
  // HTTPS
  // ----------------------------------------------------------

  WiFiClientSecure client;

  client.setInsecure();
  client.setTimeout(10000);

  HTTPClient https;

  String url =
    "https://api.telegram.org/bot";

  url +=
    TELEGRAM_BOT_TOKEN;

  url +=
    "/sendMessage";

  Serial.println(
    "HTTPS: starting Telegram API..."
  );

  if (
    !https.begin(
      client,
      url
    )
  )
  {
    Serial.println(
      "HTTPS ERROR: begin() failed"
    );

    telegramBusy = false;

    return;
  }

  https.setConnectTimeout(
    10000
  );

  https.setTimeout(
    10000
  );

  https.addHeader(
    "Content-Type",
    "application/x-www-form-urlencoded"
  );


  // ----------------------------------------------------------
  // URL ENCODE
  // ----------------------------------------------------------

  String encoded =
    message;

  encoded.replace(
    "%",
    "%25"
  );

  encoded.replace(
    " ",
    "%20"
  );

  encoded.replace(
    "\n",
    "%0A"
  );

  encoded.replace(
    "\r",
    ""
  );

  encoded.replace(
    "&",
    "%26"
  );

  encoded.replace(
    "?",
    "%3F"
  );

  encoded.replace(
    "#",
    "%23"
  );

  encoded.replace(
    "=",
    "%3D"
  );

  encoded.replace(
    "+",
    "%2B"
  );


  String body =
    "chat_id=";

  body +=
    TELEGRAM_CHAT_ID;

  body +=
    "&text=";

  body +=
    encoded;


  Serial.println(
    "Telegram: sending..."
  );

  int httpCode =
    https.POST(
      body
    );

  Serial.print(
    "Telegram HTTP code: "
  );

  Serial.println(
    httpCode
  );

  if (httpCode > 0)
  {
    String response =
      https.getString();

    Serial.print(
      "Telegram response: "
    );

    Serial.println(
      response
    );

    if (httpCode == 200)
    {
      Serial.println(
        "TELEGRAM: MESSAGE SENT SUCCESSFULLY"
      );
    }
  }
  else
  {
    Serial.print(
      "Telegram HTTPS error: "
    );

    Serial.println(
      https.errorToString(
        httpCode
      )
    );
  }

  https.end();

  telegramBusy = false;

  Serial.println(
    "======================================"
  );
}


// ============================================================
// ALERT
// ============================================================

void activateAlert(
  const String& reason
)
{
  alertActive = true;
  alertReason = reason;
  lastFallAlert = millis();
  wakeOLED();

  Serial.println();
  Serial.println("======================================");
  Serial.println("       SENTROVA ALERT ACTIVE");
  Serial.println("======================================");
  Serial.print("Reason: ");
  Serial.println(alertReason);
  Serial.println("======================================");

  if (telegramConfigValid() && WiFi.status() == WL_CONNECTED)
  {
    // Attractive, information-rich Telegram alert.
    String message = "🚨 SENTROVA OS | EMERGENCY ALERT 🚨\n\n";
    message += "⚠️ Event: ";
    message += alertReason;
    message += "\n🆘 SOS status: ";
    message += (alertReason.indexOf("SOS") >= 0) ? "ACTIVATED" : "NOT ACTIVATED";

    // Use the synchronized local clock when available.
    struct tm timeInfo;
    if (getLocalTime(&timeInfo, 100))
    {
      char timeBuffer[32];
      strftime(timeBuffer, sizeof(timeBuffer), "%d-%m-%Y %I:%M:%S %p IST", &timeInfo);
      message += "\n🕒 Time: ";
      message += timeBuffer;
    }
    else
    {
      message += "\n🕒 Time: Not synchronized";
    }

    message += "\n\n❤️ HEART RATE\n";
    if (heartRateValid)
    {
      message += "   • BPM: ";
      message += String(heartRate, 0);
      message += " BPM";
    }
    else
    {
      message += "   • BPM: No valid reading";
    }

    message += "\n🫁 SpO₂\n";
    if (spo2Valid)
    {
      message += "   • Oxygen saturation: ";
      message += String(spo2Value, 0);
      message += "%";
    }
    else
    {
      message += "   • Oxygen saturation: No valid reading";
    }

    message += "\n\n📍 LOCATION\n";
    if (gpsFix)
    {
      String mapsUrl = "https://www.google.com/maps?q=";
      mapsUrl += String(gpsLat, 6);
      mapsUrl += ",";
      mapsUrl += String(gpsLng, 6);
      message += "   • Coordinates: ";
      message += String(gpsLat, 6);
      message += ", ";
      message += String(gpsLng, 6);
      message += "\n🗺️ Open Google Maps: ";
      message += mapsUrl;
    }
    else
    {
      message += "   • GPS fix unavailable — location link not available.";
    }

    message += "\n\n🔋 Battery: ";
    message += String(batteryPercent);
    message += "%";
    message += "\n\nPlease check on the wearer and contact emergency services if needed.";
    message += "\n\n— SENTROVA OS Smartwatch";

    sendTelegramMessage(message);
  }
  else
  {
    Serial.println("Telegram alert skipped: check internet connection and Telegram configuration.");
  }
}


void clearAlert()
{
  alertActive = false;

  alertReason =
    "SYSTEM SECURE";

  wakeOLED();

  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "       SENTROVA ALERT CLEARED"
  );

  Serial.println(
    "======================================"
  );
}


// ============================================================
// I2S AUDIO: INMP441 microphone + MAX98357A speaker amplifier
// ============================================================

bool initSENTROVAAudio()
{
  i2s_config_t config = {};
  config.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_RX);
  config.sample_rate = AUDIO_SAMPLE_RATE;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  config.dma_buf_count = 4;
  config.dma_buf_len = AUDIO_BLOCK_FRAMES;
  config.use_apll = false;
  config.tx_desc_auto_clear = true;
  config.fixed_mclk = 0;

  esp_err_t err = i2s_driver_install(I2S_NUM_0, &config, 0, nullptr);
  if (err != ESP_OK)
  {
    Serial.printf("I2S driver install failed: %d\n", (int)err);
    return false;
  }

  i2s_pin_config_t pins = {};
  pins.bck_io_num = AUDIO_I2S_BCLK_PIN;
  pins.ws_io_num = AUDIO_I2S_WS_PIN;
  pins.data_out_num = AUDIO_SPK_DIN_PIN;
  pins.data_in_num = AUDIO_MIC_SD_PIN;
#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 4
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
#endif

  err = i2s_set_pin(I2S_NUM_0, &pins);
  if (err != ESP_OK)
  {
    Serial.printf("I2S pin setup failed: %d\n", (int)err);
    i2s_driver_uninstall(I2S_NUM_0);
    return false;
  }

  i2s_zero_dma_buffer(I2S_NUM_0);
  Serial.println("I2S AUDIO: READY (mic capture + local SOS speaker tones)");
  Serial.println("NOTE: ESP32-C3 cannot act as a standard A2DP Bluetooth audio speaker.");
  return true;
}

void requestSpeakerTest()
{
  if (!audioHardwareOK)
  {
    Serial.println("Speaker test unavailable: I2S audio not initialized.");
    return;
  }
  speakerTestUntil = millis() + 1800UL;
}

void serviceSENTROVAAudio()
{
  if (!audioHardwareOK) return;
  static unsigned long lastAudioBlock = 0;
  if (millis() - lastAudioBlock < 8) return;
  lastAudioBlock = millis();

  // Read microphone samples without waiting; this is a level meter only, not
  // voice recognition or a Bluetooth microphone uplink.
  static int16_t micSamples[AUDIO_BLOCK_FRAMES * 2];
  size_t bytesRead = 0;
  esp_err_t readErr = i2s_read(I2S_NUM_0, micSamples, sizeof(micSamples), &bytesRead, 0);
  if (readErr == ESP_OK && bytesRead >= sizeof(int16_t) * 2)
  {
    size_t count = bytesRead / sizeof(int16_t);
    double sum = 0.0;
    size_t n = 0;
    // INMP441 L/R pin tied low -> left channel; interleaved stereo uses even samples.
    for (size_t i = 0; i + 1 < count; i += 2)
    {
      double v = micSamples[i] / 32768.0;
      sum += v * v;
      ++n;
    }
    if (n) microphoneLevel = (float)sqrt(sum / n);
  }

  bool makeSound = (speakerAlertsEnabled && alertActive) || (millis() < speakerTestUntil);
  static int16_t txSamples[AUDIO_BLOCK_FRAMES * 2];
  static unsigned long lastToneChange = 0;
  static bool highTone = false;
  if (millis() - lastToneChange >= 250)
  {
    lastToneChange = millis();
    highTone = !highTone;
  }

  if (makeSound)
  {
    const float frequency = highTone ? 1700.0f : 1050.0f;
    const float amplitude = 5200.0f; // conservative starting volume
    for (int i = 0; i < AUDIO_BLOCK_FRAMES; ++i)
    {
      int16_t sample = (int16_t)(sinf(audioTonePhase) * amplitude);
      audioTonePhase += 2.0f * PI * frequency / AUDIO_SAMPLE_RATE;
      if (audioTonePhase >= 2.0f * PI) audioTonePhase -= 2.0f * PI;
      txSamples[i * 2] = sample;
      txSamples[i * 2 + 1] = sample;
    }
  }
  else
  {
    memset(txSamples, 0, sizeof(txSamples));
  }

  size_t bytesWritten = 0;
  i2s_write(I2S_NUM_0, txSamples, sizeof(txSamples), &bytesWritten, 0);
}


// ============================================================
// BLE CALLBACKS
// ============================================================

class SentrovaBLEServerCallbacks :
  public BLEServerCallbacks
{
public:

  void onConnect(
    BLEServer* server
  ) override
  {
    bleConnected = true;

    Serial.println();
    Serial.println(
      "================================"
    );

    Serial.println(
      "BLE: CONNECTED"
    );

    Serial.println(
      "================================"
    );
  }


  void onDisconnect(
    BLEServer* server
  ) override
  {
    bleConnected = false;

    Serial.println();
    Serial.println(
      "================================"
    );

    Serial.println(
      "BLE: DISCONNECTED"
    );

    Serial.println(
      "================================"
    );

    if (server)
    {
      server
        ->getAdvertising()
        ->start();
    }

    Serial.println(
      "BLE: ADVERTISING RESTARTED"
    );
  }
};


class SentrovaBLERXCallbacks :
  public BLECharacteristicCallbacks
{
public:

  void onWrite(
    BLECharacteristic* characteristic
  ) override
  {
    String cmd =
      characteristic->getValue();

    cmd.trim();

    if (
      cmd.length() == 0
    )
      return;

    Serial.print(
      "BLE RX: "
    );

    Serial.println(
      cmd
    );


    if (
      cmd.equalsIgnoreCase(
        "SOS"
      )
    )
    {
      activateAlert(
        "BLE SOS"
      );

      return;
    }


    if (
      cmd.equalsIgnoreCase(
        "ACK"
      ) ||
      cmd.equalsIgnoreCase(
        "CLEAR"
      )
    )
    {
      clearAlert();

      return;
    }


    if (
      cmd.equalsIgnoreCase(
        "PING"
      )
    )
    {
      Serial.println(
        "BLE: PONG"
      );

      return;
    }


    if (cmd.equalsIgnoreCase("AUDIO=ON"))
    {
      speakerAlertsEnabled = true;
      wakeOLED();
      return;
    }

    if (cmd.equalsIgnoreCase("AUDIO=OFF"))
    {
      speakerAlertsEnabled = false;
      speakerTestUntil = 0;
      wakeOLED();
      return;
    }

    if (cmd.equalsIgnoreCase("AUDIO=TEST") || cmd.equalsIgnoreCase("SPEAKER=TEST"))
    {
      requestSpeakerTest();
      wakeOLED();
      return;
    }

    if (cmd.equalsIgnoreCase("MIC=STATUS"))
    {
      if (sentrovaBLETX)
      {
        String status = String("MIC=") + (audioHardwareOK ? "READY" : "OFFLINE") + ",LEVEL=" + String(microphoneLevel, 3);
        sentrovaBLETX->setValue((uint8_t*)status.c_str(), status.length());
        sentrovaBLETX->notify();
      }
      return;
    }

    if (cmd.startsWith("NOTIFY="))
    {
      latestNotification = cmd.substring(7);
      latestNotification.trim();
      if (latestNotification.length() > 72)
        latestNotification = latestNotification.substring(0, 72);
      notificationUnread = latestNotification.length() > 0;
      wakeOLED();
      Serial.println("BLE notification preview updated");
      return;
    }

    if (
      cmd.startsWith(
        "MEDICAL="
      )
    )
    {
      String url =
        cmd.substring(8);

      url.trim();

      setMedicalURL(
        url
      );

      return;
    }
  }
};


// ============================================================
// BLE INITIALIZATION
// ============================================================

void initSENTROVABLE()
{
  BLEDevice::init(
    "SENTROVA-WATCH"
  );

  sentrovaBLEServer =
    BLEDevice::createServer();

  sentrovaBLEServer->setCallbacks(
    new SentrovaBLEServerCallbacks()
  );

  BLEService* service =
    sentrovaBLEServer->createService(
      SENTROVA_BLE_SERVICE
    );

  sentrovaBLETX =
    service->createCharacteristic(
      SENTROVA_BLE_TX,
      BLECharacteristic::PROPERTY_NOTIFY |
      BLECharacteristic::PROPERTY_READ
    );

  sentrovaBLETX->addDescriptor(
    new BLE2902()
  );

  sentrovaBLERX =
    service->createCharacteristic(
      SENTROVA_BLE_RX,
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_WRITE_NR
    );

  sentrovaBLERX->setCallbacks(
    new SentrovaBLERXCallbacks()
  );

  service->start();

  BLEAdvertising* advertising =
    BLEDevice::getAdvertising();

  advertising->addServiceUUID(
    SENTROVA_BLE_SERVICE
  );

  advertising->setScanResponse(
    true
  );

  advertising->start();

  Serial.println(
    "BLE: SENTROVA-WATCH ADVERTISING"
  );
}


// ============================================================
// BLE TELEMETRY
// ============================================================

String sentrovaTelemetryJSON()
{
  String j = "{";

  j +=
    "\"ble\":" +
    String(
      bleConnected
      ? "true"
      : "false"
    );

  j +=
    ",\"mpu\":" +
    String(
      mpuOK
      ? "true"
      : "false"
    );

  j +=
    ",\"max\":" +
    String(
      maxOK
      ? "true"
      : "false"
    );

  j +=
    ",\"acc\":" +
    String(
      accMagnitude,
      3
    );

  j +=
    ",\"gyro\":" +
    String(
      gyroMagnitude,
      3
    );

  j +=
    ",\"hr\":" +
    String(
      heartRate,
      1
    );

  j +=
    ",\"hrValid\":" +
    String(
      heartRateValid
      ? "true"
      : "false"
    );

  j +=
    ",\"spo2\":" +
    String(
      spo2Value,
      1
    );

  j +=
    ",\"spo2Valid\":" +
    String(
      spo2Valid
      ? "true"
      : "false"
    );

  j +=
    ",\"gps\":" +
    String(
      gpsFix
      ? "true"
      : "false"
    );

  j +=
    ",\"lat\":" +
    String(
      gpsLat,
      6
    );

  j +=
    ",\"lng\":" +
    String(
      gpsLng,
      6
    );

  j +=
    ",\"sat\":" +
    String(
      gpsSatellites
    );

  j +=
    ",\"fall\":" +
    String(
      aiFall
      ? "true"
      : "false"
    );

  j +=
    ",\"confidence\":" +
    String(
      aiFallProbability,
      4
    );

  j +=
    ",\"alert\":" +
    String(
      alertActive
      ? "true"
      : "false"
    );

  j +=
    ",\"reason\":\"" +
    alertReason +
    "\"";

  j +=
    ",\"batteryVoltage\":" +
    String(
      batteryVoltage,
      3
    );

  j +=
    ",\"batteryPercent\":" +
    String(
      batteryPercent
    );

  j +=
    ",\"featureWindow\":" +
    String(
      featureWindowNumber
    );

  j += "}";

  return j;
}


void sendBLETElemetry()
{
  static unsigned long lastBLE = 0;

  if (
    !bleConnected ||
    !sentrovaBLETX ||
    millis() -
    lastBLE <
    1000
  )
    return;

  lastBLE =
    millis();

  String msg =
    sentrovaTelemetryJSON();

  sentrovaBLETX->setValue(
    (uint8_t*)msg.c_str(),
    msg.length()
  );

  sentrovaBLETX->notify();
}


// ============================================================
// TOUCH
// ============================================================

void handleTouch()
{
  // TTP223 is normally active-HIGH. The SOS input is armed only after
  // stable LOW/released state, preventing boot-time/floating HIGH false SOS.
  static bool initialized = false;
  static bool sosStable = false, ackStable = false;
  static bool sosRawLast = false, ackRawLast = false;
  static unsigned long sosRawChangedAt = 0, ackRawChangedAt = 0;
  static unsigned long releasedSince = 0;
  static bool sosArmed = false;
  const unsigned long now = millis();

  const bool sosRaw = (digitalRead(SOS_PIN) == HIGH);
  const bool ackRaw = (digitalRead(ACK_PIN) == HIGH);

  if (!initialized)
  {
    initialized = true;
    sosRawLast = sosRaw; ackRawLast = ackRaw;
    sosRawChangedAt = ackRawChangedAt = now;
    sosStable = sosRaw; ackStable = ackRaw;
    releasedSince = (!sosRaw) ? now : 0;
    Serial.printf("TTP223 startup: SOS=%s ACK=%s; wait for SOS release to arm.\n",
                  sosRaw ? "HIGH/touched" : "LOW/released",
                  ackRaw ? "HIGH/touched" : "LOW/released");
  }

  if (sosRaw != sosRawLast) { sosRawLast = sosRaw; sosRawChangedAt = now; }
  if (ackRaw != ackRawLast) { ackRawLast = ackRaw; ackRawChangedAt = now; }
  if (now - sosRawChangedAt >= 40UL) sosStable = sosRaw;
  if (now - ackRawChangedAt >= 40UL) ackStable = ackRaw;

  if (!sosStable)
  {
    if (releasedSince == 0) releasedSince = now;
    if (!sosArmed && now - releasedSince >= 500UL)
    {
      sosArmed = true;
      Serial.println("SOS touch: ARMED");
    }
  }
  else releasedSince = 0;

  sosTouched = sosStable;
  ackTouched = ackStable;
  if (sosStable || ackStable) wakeOLED();

  // SOS requires a fresh touch and a continuous 1.5-second hold.
  if (sosStable && sosArmed)
  {
    if (sosStart == 0) { sosStart = now; sosHandled = false; }
    if (!sosHandled && now - sosStart >= 1500UL)
    {
      activateAlert("MANUAL TOUCH SOS");
      sosHandled = true;
      sosArmed = false; // release is required before another activation
      Serial.println("SOS: manual touch hold accepted.");
    }
  }
  else if (!sosStable)
  {
    if (sosStart != 0 && !sosHandled && now - sosStart < 1500UL && watchPage == 8)
    {
      speakerAlertsEnabled = !speakerAlertsEnabled;
      if (!speakerAlertsEnabled) speakerTestUntil = 0;
      Serial.printf("SOS speaker alerts: %s\n", speakerAlertsEnabled ? "ENABLED" : "MUTED");
      wakeOLED();
    }
    sosStart = 0;
    sosHandled = false;
  }

  // ACK short tap advances page; 1.5-second hold clears an active alert.
  if (ackStable)
  {
    if (ackStart == 0) { ackStart = now; ackHandled = false; }
    if (!ackHandled && now - ackStart >= 1500UL)
    {
      if (alertActive) clearAlert();
      ackHandled = true;
    }
  }
  else
  {
    if (ackStart != 0 && !ackHandled && now - ackStart < 1500UL)
    {
      watchPage++;
      if (watchPage >= TOTAL_PAGES) watchPage = 0;
      wakeOLED();
    }
    ackStart = 0;
    ackHandled = false;
  }
}


// ============================================================
// SENTROVA OS — FIGMA-INSPIRED MONOCHROME OLED UI
// Adapted for SH1106 128x64. No browser assets/fonts are used.
// ============================================================

void header(const char* title)
{
  oled.setFont(u8g2_font_5x8_tf);
  oled.drawStr(3, 8, title);

  // Compact live status icons: BLE, battery and a divider.
  if (bleConnected) oled.drawDisc(103, 5, 2);
  else oled.drawCircle(103, 5, 2);

  oled.drawFrame(111, 2, 12, 6);
  oled.drawBox(123, 4, 2, 2);
  int bars = constrain((batteryPercent + 24) / 25, 0, 4);
  if (bars > 0) oled.drawBox(113, 4, bars * 2, 2);

  oled.drawHLine(0, 11, 128);
}

void drawHome()
{
  oled.clearBuffer();
  if (alertActive)
  {
    oled.setFont(u8g2_font_7x14B_tf);
    oled.drawStr(24, 17, "SOS / ALERT");
    oled.drawHLine(0, 21, 128);
    oled.setFont(u8g2_font_5x8_tf);
    String reason = alertReason;
    if (reason.length() > 20) reason = reason.substring(0, 20);
    oled.drawStr(4, 35, reason.c_str());
    oled.drawStr(4, 49, "HOLD ACK: CLEAR ALERT");
    oled.drawStr(4, 61, "SENSORS STILL ACTIVE");
    oled.sendBuffer();
    return;
  }

  header("SENTROVA OS");

  // Prefer internet-synchronized NTP time in IST; fall back to GPS time.
  // Do not show uptime as a real clock.
  oled.setFont(u8g2_font_logisoso24_tf);
  struct tm timeInfo;
  bool haveTime = getLocalTime(&timeInfo, 5) && timeInfo.tm_year >= (2024 - 1900);

  char timeText[8];
  if (haveTime)
  {
    snprintf(timeText, sizeof(timeText), "%02d:%02d", timeInfo.tm_hour, timeInfo.tm_min);
    int w = oled.getStrWidth(timeText);
    oled.drawStr((128 - w) / 2, 39, timeText);
  }
  else if (gps.time.isValid())
  {
    int h = (gps.time.hour() + 5) % 24;
    int m = gps.time.minute() + 30;
    if (m >= 60) { m -= 60; h = (h + 1) % 24; }
    snprintf(timeText, sizeof(timeText), "%02d:%02d", h, m);
    int w = oled.getStrWidth(timeText);
    oled.drawStr((128 - w) / 2, 39, timeText);
  }
  else
  {
    oled.setFont(u8g2_font_7x14B_tf);
    oled.drawStr(27, 34, "SYNCING TIME");
  }

  oled.setFont(u8g2_font_5x8_tf);
  oled.drawHLine(5, 44, 118);
  oled.drawDisc(8, 54, 3);
  oled.drawStr(16, 57, alertActive ? "ALERT ACTIVE" : "MONITORING ACTIVE");
  char batteryText[12];
  snprintf(batteryText, sizeof(batteryText), "%d%%", batteryPercent);
  int bw = oled.getStrWidth(batteryText);
  oled.drawStr(124 - bw, 57, batteryText);
  oled.drawStr(4, 63, "ACK: NEXT PAGE");
  oled.sendBuffer();
}

void drawMenu()
{
  oled.clearBuffer();
  header("MENU  /  02");
  const char* labels[4] = {"HOME", "HEALTH", "SAFETY", "SETTINGS"};
  for (int i = 0; i < 4; ++i)
  {
    int x = (i % 2) * 63 + 2;
    int y = (i / 2) * 24 + 15;
    oled.drawFrame(x, y, 60, 21);
    if (i == 1) oled.drawDisc(x + 7, y + 7, 2);
    else if (i == 2) oled.drawTriangle(x + 4, y + 3, x + 10, y + 3, x + 7, y + 10);
    else if (i == 3) oled.drawCircle(x + 7, y + 7, 3);
    else oled.drawBox(x + 4, y + 4, 6, 6);
    oled.setFont(u8g2_font_5x8_tf);
    oled.drawStr(x + 15, y + 13, labels[i]);
  }
  oled.drawStr(22, 62, "ACK TO CYCLE SCREENS");
  oled.sendBuffer();
}

void drawHealth()
{
  oled.clearBuffer();
  header("HEALTH  /  03");

  oled.setFont(u8g2_font_5x8_tf);
  oled.drawFrame(2, 15, 60, 29);
  oled.drawStr(7, 23, "HEART RATE");
  oled.setFont(u8g2_font_7x14B_tf);
  if (heartRateValid) { oled.setCursor(7, 39); oled.print(heartRate, 0); }
  else oled.drawStr(7, 39, "--");
  oled.setFont(u8g2_font_5x8_tf);
  oled.drawStr(38, 39, "BPM");

  oled.drawFrame(66, 15, 60, 29);
  oled.drawStr(71, 23, "SpO2");
  oled.setFont(u8g2_font_7x14B_tf);
  if (spo2Valid) { oled.setCursor(71, 39); oled.print(spo2Value, 0); oled.print("%"); }
  else oled.drawStr(71, 39, "--%");

  oled.setFont(u8g2_font_5x8_tf);
  oled.drawStr(4, 55, fingerDetected ? "FINGER DETECTED" : "PLACE FINGER ON SENSOR");
  oled.drawStr(4, 63, maxOK ? "MAX30102 READY" : "MAX30102 OFFLINE");
  oled.sendBuffer();
}

void drawSafety()
{
  oled.clearBuffer();
  header("SAFETY  /  04");

  if (alertActive)
  {
    oled.drawFrame(3, 15, 122, 26);
    oled.setFont(u8g2_font_7x14B_tf);
    oled.drawStr(12, 33, "ALERT ACTIVE");
    oled.setFont(u8g2_font_5x8_tf);
    String reason = alertReason;
    if (reason.length() > 19) reason = reason.substring(0, 19);
    oled.drawStr(4, 51, reason.c_str());
    oled.drawStr(4, 62, "HOLD ACK TO CLEAR");
  }
  else
  {
    oled.drawCircle(15, 27, 9);
    oled.drawLine(10, 27, 14, 31);
    oled.drawLine(14, 31, 21, 22);
    oled.setFont(u8g2_font_7x14B_tf);
    oled.drawStr(31, 31, "PROTECTED");
    oled.setFont(u8g2_font_5x8_tf);
    oled.drawStr(4, 45, aiFall ? "DNN: FALL RISK" : "DNN: MONITORING");
    char prob[24];
    snprintf(prob, sizeof(prob), "FALL PROB: %d%%", (int)(aiFallProbability * 100.0f));
    oled.drawStr(4, 55, prob);
    oled.drawStr(4, 63, "HOLD SOS 1.5s TO ALERT");
  }
  oled.sendBuffer();
}

void drawSettings()
{
  oled.clearBuffer();
  header("SETTINGS  /  05");
  oled.setFont(u8g2_font_5x8_tf);
  oled.drawStr(4, 22, "BLUETOOTH");
  oled.drawStr(83, 22, bleConnected ? "CONNECTED" : "ADVERTISING");
  oled.drawHLine(3, 25, 122);
  oled.drawStr(4, 35, "INTERNET");
  oled.drawStr(83, 35, WiFi.status() == WL_CONNECTED ? "ONLINE" : "OFFLINE");
  oled.drawHLine(3, 38, 122);
  oled.drawStr(4, 48, "MPU-6500");
  oled.drawStr(83, 48, mpuOK ? "READY" : "OFFLINE");
  oled.drawStr(4, 59, "BATTERY");
  oled.setCursor(83, 59); oled.print(batteryVoltage, 2); oled.print("V");
  oled.sendBuffer();
}

void drawNotifications()
{
  oled.clearBuffer();
  header(notificationUnread ? "NOTIFICATIONS *" : "NOTIFICATIONS");
  oled.setFont(u8g2_font_6x10_tf);
  if (latestNotification.length() == 0)
  {
    oled.drawStr(5, 29, "NO PHONE NOTIFICATIONS");
    oled.setFont(u8g2_font_5x8_tf);
    oled.drawStr(5, 43, "Connect phone over BLE");
    oled.drawStr(5, 57, "Send NOTIFY=message");
  }
  else
  {
    oled.drawStr(5, 25, "PHONE PREVIEW");
    String first = latestNotification.substring(0, min((int)latestNotification.length(), 20));
    oled.drawStr(5, 38, first.c_str());
    if (latestNotification.length() > 20)
    {
      String second = latestNotification.substring(20, min((int)latestNotification.length(), 40));
      oled.drawStr(5, 49, second.c_str());
    }
    if (latestNotification.length() > 40)
    {
      String third = latestNotification.substring(40, min((int)latestNotification.length(), 60));
      oled.drawStr(5, 60, third.c_str());
    }
    notificationUnread = false;
  }
  oled.sendBuffer();
}

void drawGPS()
{
  oled.clearBuffer();
  header("LOCATION  /  07");
  oled.setFont(u8g2_font_6x10_tf);
  if (gpsFix)
  {
    oled.drawStr(4, 24, "GPS FIX ACQUIRED");
    oled.setFont(u8g2_font_5x8_tf);
    oled.setCursor(4, 37); oled.print("SATELLITES: "); oled.print(gpsSatellites);
    oled.setCursor(4, 48); oled.print(gpsLat, 4);
    oled.setCursor(68, 48); oled.print(gpsLng, 4);
    oled.drawStr(4, 61, "LOCATION AVAILABLE");
  }
  else
  {
    oled.drawStr(4, 25, "SEARCHING FOR GPS");
    oled.setFont(u8g2_font_5x8_tf);
    oled.setCursor(4, 40); oled.print("SATELLITES: "); oled.print(gpsSatellites);
    oled.drawStr(4, 54, "TRY OUTDOORS");
    oled.drawStr(4, 63, "NO FIX YET");
  }
  oled.sendBuffer();
}

void drawAudioBluetooth()
{
  oled.clearBuffer();
  header("AUDIO / BLUETOOTH");
  oled.setFont(u8g2_font_6x10_tf);
  oled.drawStr(4, 24, audioHardwareOK ? "AUDIO HARDWARE READY" : "AUDIO HARDWARE OFFLINE");
  oled.setFont(u8g2_font_5x8_tf);
  oled.drawStr(4, 35, speakerAlertsEnabled ? "SOS SPEAKER: ENABLED" : "SOS SPEAKER: MUTED");
  oled.drawStr(4, 45, bleConnected ? "BLE: PHONE CONNECTED" : "BLE: ADVERTISING");
  char micText[28];
  snprintf(micText, sizeof(micText), "MIC LEVEL: %d%%", (int)constrain(microphoneLevel * 1000.0f, 0.0f, 100.0f));
  oled.drawStr(4, 54, micText);
  oled.drawStr(4, 63, "SHORT SOS: TOGGLE ALERT SOUND");
  oled.sendBuffer();
}

void drawWatch()
{
  switch (watchPage)
  {
    case 0: drawHome(); break;
    case 1: drawMenu(); break;
    case 2: drawHealth(); break;
    case 3: drawSafety(); break;
    case 4: drawSettings(); break;
    case 5: drawNotifications(); break;
    case 6: drawGPS(); break;
    case 7: drawMedicalQR(); break;
    case 8: drawAudioBluetooth(); break;
    default: watchPage = 0; drawHome(); break;
  }
}


// ============================================================
// WEB STATUS
// ============================================================

void handleStatus()
{
  String json = "{";

  json +=
    "\"alert\":";

  json +=
    alertActive
    ? "true"
    : "false";

  json +=
    ",\"reason\":\"";

  json +=
    alertReason;

  json += "\"";

  json +=
    ",\"acc\":" +
    String(
      accMagnitude,
      2
    );

  json +=
    ",\"gyro\":" +
    String(
      gyroMagnitude,
      2
    );

  json +=
    ",\"ir\":" +
    String(
      irValue
    );

  json +=
    ",\"red\":" +
    String(
      redValue
    );

  json +=
    ",\"finger\":" +
    String(
      fingerDetected
      ? "true"
      : "false"
    );

  json +=
    ",\"gps\":" +
    String(
      gpsFix
      ? "true"
      : "false"
    );

  json +=
    ",\"lat\":" +
    String(
      gpsLat,
      6
    );

  json +=
    ",\"lng\":" +
    String(
      gpsLng,
      6
    );

  json +=
    ",\"sat\":" +
    String(
      gpsSatellites
    );

  json +=
    ",\"sos\":" +
    String(
      sosTouched
      ? "true"
      : "false"
    );

  json +=
    ",\"ack\":" +
    String(
      ackTouched
      ? "true"
      : "false"
    );

  json +=
    ",\"aiFall\":" +
    String(
      aiFall
      ? "true"
      : "false"
    );

  json +=
    ",\"confidence\":" +
    String(
      aiFallProbability,
      4
    );

  json +=
    ",\"hr\":" +
    String(
      heartRate,
      1
    );

  json +=
    ",\"hrValid\":" +
    String(
      heartRateValid
      ? "true"
      : "false"
    );

  json +=
    ",\"spo2\":" +
    String(
      spo2Value,
      1
    );

  json +=
    ",\"spo2Valid\":" +
    String(
      spo2Valid
      ? "true"
      : "false"
    );

  json +=
    ",\"batteryVoltage\":" +
    String(
      batteryVoltage,
      3
    );

  json +=
    ",\"batteryPercent\":" +
    String(
      batteryPercent
    );

  json +=
    ",\"ble\":" +
    String(
      bleConnected
      ? "true"
      : "false"
    );

  json +=
    ",\"featureWindow\":" +
    String(
      featureWindowNumber
    );

  json +=
    ",\"telegram\":" +
    String(
      telegramConfigValid() &&
      WiFi.status() ==
      WL_CONNECTED
      ? "true"
      : "false"
    );

  json +=
    ",\"medicalURL\":\"";

  json +=
    medicalURL;

  json += "\"";

  json += "}";

  server.send(
    200,
    "application/json",
    json
  );
}


// ============================================================
// FEATURE API
// ============================================================

void handleFeatureAPI()
{
  String json = "[";

  for (int i = 0; i < 60; i++)
  {
    if (i)
      json += ",";

    json +=
      String(
        featureVector[i],
        8
      );
  }

  json += "]";

  server.send(
    200,
    "application/json",
    json
  );
}


// ============================================================
// MEDICAL API
// ============================================================

void handleMedicalURL()
{
  if (!server.hasArg("url"))
  {
    server.send(
      400,
      "text/plain",
      "Missing url"
    );

    return;
  }

  String url =
    server.arg("url");

  if (
    !setMedicalURL(url)
  )
  {
    server.send(
      400,
      "text/plain",
      "Invalid URL or URL too long"
    );

    return;
  }

  wakeOLED();

  server.send(
    200,
    "application/json",
    "{\"ok\":true,\"message\":\"Medical URL saved and QR ready\"}"
  );
}


// ============================================================
// WEB SOS / ACK
// ============================================================

void handleWebSOS()
{
  activateAlert(
    "WEB SOS ACTIVATED"
  );

  server.send(
    200,
    "text/plain",
    "SOS ACTIVATED"
  );
}


void handleWebACK()
{
  clearAlert();

  server.send(
    200,
    "text/plain",
    "ALERT CLEARED"
  );
}


// ============================================================
// DASHBOARD
// ============================================================

String dashboard()
{
  String html = R"HTML(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport"
content="width=device-width,initial-scale=1">

<title>SENTROVA OS</title>

<style>

*{
box-sizing:border-box;
}

body{
margin:0;
font-family:Arial,sans-serif;
background:#06090f;
color:#fff;
}

.header{
padding:25px;
text-align:center;
border-bottom:1px solid #253244;
}

.logo{
font-size:30px;
font-weight:800;
letter-spacing:3px;
}

.subtitle{
color:#8d9aad;
margin-top:7px;
}

.container{
max-width:1100px;
margin:auto;
padding:20px;
}

.status{
padding:22px;
border-radius:22px;
margin-bottom:20px;
background:#102619;
border:1px solid #214a31;
text-align:center;
}

.status.alert{
background:#321216;
border-color:#6b242d;
}

.statusValue{
font-size:35px;
font-weight:800;
margin:7px;
}

.safe{
color:#42e58b;
}

.alertText{
color:#ff5361;
}

.grid{
display:grid;
grid-template-columns:
repeat(auto-fit,minmax(210px,1fr));
gap:15px;
}

.card{
padding:20px;
border-radius:18px;
background:#111925;
border:1px solid #263244;
}

.title{
font-size:13px;
color:#8d9aad;
text-transform:uppercase;
}

.value{
font-size:27px;
font-weight:700;
margin-top:10px;
}

.info{
font-size:13px;
color:#7f8da0;
margin-top:6px;
}

.controls{
margin-top:20px;
display:grid;
grid-template-columns:1fr 1fr;
gap:15px;
}

button{
padding:17px;
border:0;
border-radius:14px;
font-size:16px;
font-weight:700;
cursor:pointer;
}

.sos{
background:#e83d4e;
color:white;
}

.ack{
background:#39dc86;
color:#07120b;
}

input{
width:100%;
padding:14px;
border-radius:12px;
border:1px solid #33445a;
background:#0a101a;
color:#fff;
margin-top:10px;
}

.blue{
background:#4c8dff;
color:#fff;
margin-top:10px;
width:100%;
}

</style>
</head>

<body>

<div class="header">

<div class="logo">
SENTROVA
</div>

<div class="subtitle">
SMART SAFETY & HEALTH OS
</div>

</div>

<div class="container">

<div id="statusBox"
class="status">

<div>SYSTEM STATUS</div>

<div id="status"
class="statusValue safe">
SECURE
</div>

<div id="reason">
All systems operating normally
</div>

</div>

<div class="grid">

<div class="card">
<div class="title">Motion</div>
<div id="acc" class="value">0.00 g</div>
<div class="info">Acceleration</div>
</div>

<div class="card">
<div class="title">Gyroscope</div>
<div id="gyro" class="value">0.00</div>
<div class="info">rad/s</div>
</div>

<div class="card">
<div class="title">AI DNN</div>
<div id="ai" class="value">READY</div>
<div id="confidence" class="info">Confidence: 0%</div>
</div>

<div class="card">
<div class="title">Health</div>
<div id="health" class="value">-- / --</div>
<div class="info">HR / SpO₂</div>
</div>

<div class="card">
<div class="title">GPS</div>
<div id="gps" class="value">SEARCHING</div>
<div id="coords" class="info">Waiting for fix</div>
</div>

<div class="card">
<div class="title">Battery</div>
<div id="battery" class="value">--%</div>
<div id="batteryV" class="info">-- V</div>
</div>

<div class="card">
<div class="title">BLE</div>
<div id="ble" class="value">OFFLINE</div>
</div>

<div class="card">
<div class="title">Telegram</div>
<div id="telegram" class="value">OFFLINE</div>
</div>

<div class="card"
style="grid-column:1/-1">

<div class="title">
Medical ID / QR
</div>

<input
id="medicalURL"
type="url"
placeholder="https://example.com/medical/..."
>

<button
class="blue"
onclick="saveMedicalURL()">
GENERATE / SEND TO WATCH
</button>

<div id="medicalStatus"
class="info">
Enter a short URL. Maximum 67 characters.
</div>

<a
id="medicalLink"
href="#"
target="_blank"
style="color:#70a7ff;word-break:break-all">
</a>

</div>

</div>

<div class="controls">

<button
class="sos"
onclick="fetch('/api/sos')">
ACTIVATE SOS
</button>

<button
class="ack"
onclick="fetch('/api/ack')">
CLEAR ALERT
</button>

</div>

</div>

<script>

async function updateData(){

try{

const response =
await fetch('/api/status');

const d =
await response.json();

const status =
document.getElementById("status");

const box =
document.getElementById("statusBox");

if(d.alert){

status.innerText =
"ALERT";

status.className =
"statusValue alertText";

box.className =
"status alert";

}else{

status.innerText =
"SECURE";

status.className =
"statusValue safe";

box.className =
"status";

}

document.getElementById("reason")
.innerText =
d.reason;

document.getElementById("acc")
.innerText =
d.acc.toFixed(2)+" g";

document.getElementById("gyro")
.innerText =
d.gyro.toFixed(2);

document.getElementById("ai")
.innerText =
d.aiFall ? "FALL" : "ADL";

document.getElementById("confidence")
.innerText =
"Confidence: "+
(d.confidence*100).toFixed(1)+
"%";

document.getElementById("health")
.innerText =
(d.hrValid ?
d.hr.toFixed(0) :
"--")+
" / "+
(d.spo2Valid ?
d.spo2.toFixed(0) :
"--");

document.getElementById("gps")
.innerText =
d.gps ?
"FIX" :
"SEARCHING";

if(d.gps){

document.getElementById("coords")
.innerText =
d.lat.toFixed(6)+
", "+
d.lng.toFixed(6);

}else{

document.getElementById("coords")
.innerText =
"Waiting for fix";

}

document.getElementById("battery")
.innerText =
d.batteryPercent+"%";

document.getElementById("batteryV")
.innerText =
d.batteryVoltage.toFixed(2)+" V";

document.getElementById("ble")
.innerText =
d.ble ?
"CONNECTED" :
"OFFLINE";

document.getElementById("telegram")
.innerText =
d.telegram ?
"ONLINE" :
"OFFLINE";

if(d.medicalURL){

document.getElementById(
"medicalURL"
).value =
d.medicalURL;

document.getElementById(
"medicalLink"
).href =
d.medicalURL;

document.getElementById(
"medicalLink"
).innerText =
d.medicalURL;

}

}catch(e){}

}


async function saveMedicalURL(){

const url =
document
.getElementById("medicalURL")
.value
.trim();

const status =
document
.getElementById("medicalStatus");

if(!/^https?:\/\//i.test(url)){

status.innerText =
"URL must start with http:// or https://";

return;

}

if(url.length > 67){

status.innerText =
"URL too long. Maximum 67 characters.";

return;

}

try{

const body =
"url="+
encodeURIComponent(url);

const response =
await fetch(
"/api/medical",
{
method:"POST",
headers:{
"Content-Type":
"application/x-www-form-urlencoded"
},
body:body
}
);

const result =
await response.json();

status.innerText =
result.message ||
"Saved";

document.getElementById(
"medicalLink"
).href =
url;

document.getElementById(
"medicalLink"
).innerText =
url;

}catch(e){

status.innerText =
"Failed to send URL";

}

}


setInterval(
updateData,
1000
);

updateData();

</script>

</body>
</html>
)HTML";

  return html;
}


// ============================================================
// WIFI
// ============================================================

const char* wifiStatusText(wl_status_t status)
{
  switch (status)
  {
    case WL_IDLE_STATUS:     return "IDLE";
    case WL_NO_SSID_AVAIL:   return "SSID NOT FOUND";
    case WL_SCAN_COMPLETED:  return "SCAN COMPLETED";
    case WL_CONNECTED:       return "CONNECTED";
    case WL_CONNECT_FAILED:  return "CONNECT FAILED (CHECK PASSWORD)";
    case WL_CONNECTION_LOST: return "CONNECTION LOST";
    case WL_DISCONNECTED:    return "DISCONNECTED";
    default:                 return "UNKNOWN";
  }
}


void startWiFi()
{
  Serial.println();
  Serial.println(
    "Starting SENTROVA WiFi..."
  );

  WiFi.mode(
    WIFI_AP_STA
  );

  WiFi.setAutoReconnect(
    true
  );

  WiFi.persistent(
    false
  );


  // ----------------------------------------------------------
  // LOCAL AP
  // ----------------------------------------------------------

  WiFi.softAPConfig(
    apIP,
    gateway,
    subnet
  );

  bool apResult =
    WiFi.softAP(
      WIFI_SSID,
      WIFI_PASS
    );

  if (apResult)
  {
    Serial.println(
      "LOCAL AP     : OK"
    );

    Serial.print(
      "AP SSID      : "
    );

    Serial.println(
      WIFI_SSID
    );

    Serial.print(
      "AP IP        : "
    );

    Serial.println(
      WiFi.softAPIP()
    );
  }
  else
  {
    Serial.println(
      "LOCAL AP     : FAILED"
    );
  }


  // ----------------------------------------------------------
  // INTERNET STA
  // ----------------------------------------------------------

  Serial.println();
  Serial.println(
    "INTERNET WIFI"
  );

  Serial.println("STA target   : configured (SSID hidden)");
  Serial.println("Wi-Fi band   : 2.4 GHz required on ESP32-C3");

  WiFi.begin(
    INTERNET_SSID,
    INTERNET_PASS
  );

  unsigned long start =
    millis();

  Serial.print(
    "Connecting"
  );

  while (
    WiFi.status() !=
    WL_CONNECTED &&
    millis() -
    start <
    20000UL
  )
  {
    delay(500);

    Serial.print(
      "."
    );
  }

  Serial.println();

  if (
    WiFi.status() ==
    WL_CONNECTED
  )
  {
    Serial.println(
      "INTERNET WIFI : CONNECTED"
    );

    Serial.print(
      "STA IP        : "
    );

    Serial.println(
      WiFi.localIP()
    );

    Serial.print(
      "Gateway       : "
    );

    Serial.println(
      WiFi.gatewayIP()
    );

    Serial.print(
      "DNS           : "
    );

    Serial.println(
      WiFi.dnsIP()
    );

    Serial.print(
      "RSSI          : "
    );

    Serial.print(
      WiFi.RSSI()
    );

    Serial.println(
      " dBm"
    );

    telegramEnabled =
      telegramConfigValid();
  }
  else
  {
    Serial.println(
      "INTERNET WIFI : FAILED"
    );

    Serial.print(
      "WiFi status   : "
    );

    Serial.printf("WiFi status   : %d (%s)\n",
                  (int)WiFi.status(), wifiStatusText(WiFi.status()));
    Serial.println("Check: exact SSID/password, 2.4 GHz, DHCP, router MAC filtering.");
    Serial.println("The local SENTROVA-SETUP AP does not itself provide internet.");

    telegramEnabled = false;
  }

  Serial.println();

  Serial.print(
    "TELEGRAM      : "
  );

  Serial.println(
    telegramConfigValid()
    ? "CONFIGURED"
    : "NOT CONFIGURED"
  );
}


// ============================================================
// WIFI RECONNECTION
// ============================================================

void maintainNtpTime()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    ntpConfigured = false;
    return;
  }

  if (!ntpConfigured)
  {
    // POSIX timezone signs are reversed: IST is UTC+05:30, hence IST-5:30.
    configTzTime("IST-5:30", "pool.ntp.org", "time.google.com", "time.cloudflare.com");
    ntpConfigured = true;
    Serial.println("TIME: NTP requested; timezone set to IST (UTC+05:30)");
  }

  if (millis() - lastNtpStatusLog >= 15000UL)
  {
    lastNtpStatusLog = millis();
    time_t now = time(nullptr);
    if (now > 1704067200)
    {
      struct tm timeInfo;
      localtime_r(&now, &timeInfo);
      Serial.printf("TIME: IST synchronized: %04d-%02d-%02d %02d:%02d:%02d\n",
                    timeInfo.tm_year + 1900, timeInfo.tm_mon + 1, timeInfo.tm_mday,
                    timeInfo.tm_hour, timeInfo.tm_min, timeInfo.tm_sec);
    }
    else
    {
      Serial.println("TIME: waiting for NTP synchronization...");
    }
  }
}

void maintainInternetWiFi()
{
  static unsigned long lastCheck = 0;
  static bool connectionAttemptActive = false;
  static unsigned long attemptStarted = 0;
  const unsigned long now = millis();

  if (WiFi.status() == WL_CONNECTED)
  {
    connectionAttemptActive = false;
    telegramEnabled = telegramConfigValid();
    return;
  }

  telegramEnabled = false;
  if (now - lastCheck < 15000UL) return;
  lastCheck = now;

  Serial.printf("Wi-Fi retry: status %d (%s)\n",
                (int)WiFi.status(), wifiStatusText(WiFi.status()));
  if (!connectionAttemptActive || now - attemptStarted > 20000UL)
  {
    Serial.println("Wi-Fi retry: starting non-blocking reconnect (credentials hidden)");
    WiFi.disconnect(false, false);
    WiFi.begin(INTERNET_SSID, INTERNET_PASS);
    attemptStarted = now;
    connectionAttemptActive = true;
  }
}


// ============================================================
// WEB SERVER
// ============================================================

void startWebServer()
{
  server.on(
    "/",
    HTTP_GET,
    []()
    {
      server.send(
        200,
        "text/html",
        dashboard()
      );
    }
  );

  server.on(
    "/api/status",
    HTTP_GET,
    handleStatus
  );

  server.on(
    "/api/sos",
    HTTP_GET,
    handleWebSOS
  );

  server.on(
    "/api/ack",
    HTTP_GET,
    handleWebACK
  );

  server.on(
    "/api/medical",
    HTTP_POST,
    handleMedicalURL
  );

  server.on(
    "/api/features",
    HTTP_GET,
    handleFeatureAPI
  );

  server.begin();

  Serial.println(
    "WEB SERVER : ONLINE"
  );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(
    115200
  );

  delay(1500);

  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "          SENTROVA OS"
  );

  Serial.println(
    "      SMARTWATCH EDITION"
  );

  Serial.println(
    "======================================"
  );


  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  Wire.begin(
    SDA_PIN,
    SCL_PIN
  );

  // Start at standard-mode speed for reliable multi-device breadboard wiring.
  Wire.setClock(100000);

  // Separate hardware I2C controller for MPU-6500 on GPIO6/GPIO7.



  // ----------------------------------------------------------
  // TOUCH
  // ----------------------------------------------------------

  pinMode(SOS_PIN, INPUT_PULLDOWN);
  pinMode(ACK_PIN, INPUT_PULLDOWN);


  // ----------------------------------------------------------
  // BATTERY ADC
  // ----------------------------------------------------------

  analogReadResolution(
    12
  );

  analogSetPinAttenuation(
    BATTERY_ADC_PIN,
    ADC_11db
  );


  // ----------------------------------------------------------
  // OLED
  // ----------------------------------------------------------

  Serial.println(
    "OLED       : STARTING"
  );

  oled.begin();

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_7x14B_tf
  );

  oled.drawStr(
    12,
    25,
    "SENTROVA"
  );

  oled.setFont(
    u8g2_font_5x8_tf
  );

  oled.drawStr(
    28,
    42,
    "SMARTWATCH OS"
  );

  oled.drawStr(
    35,
    56,
    "BOOTING..."
  );

  oled.sendBuffer();

  delay(1200);


  // ----------------------------------------------------------
  // I2S AUDIO
  // ----------------------------------------------------------

  Serial.println("AUDIO      : STARTING");
  audioHardwareOK = initSENTROVAAudio();
  Serial.println(audioHardwareOK ? "AUDIO      : OK" : "AUDIO      : FAILED");


  // ----------------------------------------------------------
  // MPU
  // ----------------------------------------------------------

  Serial.println("MPU-6500   : STARTING");
  Serial.println("MPU wiring : configured for SDA=GPIO6, SCL=GPIO7");
  Serial.println("MPU check  : verify VCC, GND, SDA/SCL and address 0x68/0x69");

  mpuOK =
    initializeMPU();

  Serial.print(
    "MPU-6500   : "
  );

  Serial.println(
    mpuOK
    ? "OK"
    : "FAILED"
  );


  // ----------------------------------------------------------
  // MAX30102
  // ----------------------------------------------------------

  Serial.println(
    "MAX30102   : STARTING"
  );

  maxOK =
    initializeMAX30102();

  Serial.print(
    "MAX30102   : "
  );

  Serial.println(
    maxOK
    ? "OK"
    : "FAILED"
  );


  // ----------------------------------------------------------
  // BATTERY
  // ----------------------------------------------------------

  readBattery();


  // ----------------------------------------------------------
  // BLE
  // ----------------------------------------------------------

  initSENTROVABLE();


  // ----------------------------------------------------------
  // PREFERENCES
  // ----------------------------------------------------------

  loadPersistentSettings();


  // ----------------------------------------------------------
  // GPS
  // ----------------------------------------------------------

  GPSserial.begin(
    9600,
    SERIAL_8N1,
    GPS_RX_PIN,
    GPS_TX_PIN
  );

  Serial.println(
    "GPS        : UART ONLINE"
  );


  // ----------------------------------------------------------
  // WIFI
  // ----------------------------------------------------------

  startWiFi();


  // ----------------------------------------------------------
  // WEB
  // ----------------------------------------------------------

  startWebServer();


  // ----------------------------------------------------------
  // READY
  // ----------------------------------------------------------

  oled.clearBuffer();

  oled.setFont(
    u8g2_font_6x13B_tf
  );

  oled.drawStr(
    25,
    17,
    "SENTROVA"
  );

  oled.setFont(
    u8g2_font_5x8_tf
  );

  oled.drawStr(
    19,
    30,
    "SYSTEM READY"
  );

  oled.drawStr(
    17,
    42,
    "WIFI: ONLINE"
  );

  oled.drawStr(
    24,
    53,
    "192.168.4.1"
  );

  oled.drawStr(
    40,
    63,
    "READY"
  );

  oled.sendBuffer();

  delay(2000);

  bootTime =
    millis();

  lastUserActivity =
    millis();

  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "          SENTROVA READY"
  );

  Serial.println(
    "======================================"
  );

  Serial.println(
    "Local WiFi : SENTROVA-SETUP"
  );
  Serial.println("AP password: configured in firmware (not printed)");
Serial.println(
    "Dashboard  : http://192.168.4.1"
  );

  Serial.println(
    "Medical QR : OLED PAGE 6"
  );

  Serial.println(
    "Telegram   : Internet STA"
  );

  Serial.println(
    "======================================"
  );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  unsigned long now =
    millis();


  // ----------------------------------------------------------
  // INTERNET WIFI MAINTENANCE
  // ----------------------------------------------------------

  maintainInternetWiFi();
  maintainNtpTime();


  // ----------------------------------------------------------
  // GPS
  // ----------------------------------------------------------

  updateGPS();


  // ----------------------------------------------------------
  // TOUCH
  // ----------------------------------------------------------

  handleTouch();


  // ----------------------------------------------------------
  // MPU + 60 FEATURE ENGINE
  // 20 Hz
  // ----------------------------------------------------------

  static unsigned long lastMotion = 0;

  if (
    now -
    lastMotion >=
    50
  )
  {
    lastMotion =
      now;

    if (mpuOK)
    {
      if (readMPU())
        processFeatureSample();
    }
  }


  // ----------------------------------------------------------
  // MAX30102
  // Approximately 100 Hz
  // ----------------------------------------------------------

  static unsigned long lastMAX = 0;

  if (
    now -
    lastMAX >=
    10
  )
  {
    lastMAX =
      now;

    if (maxOK)
      updateMAXHealthBuffer();
  }


  // ----------------------------------------------------------
  // BATTERY
  // ----------------------------------------------------------

  static unsigned long lastBattery = 0;

  if (
    now -
    lastBattery >=
    1000
  )
  {
    lastBattery =
      now;

    readBattery();

    if (
      batteryLow &&
      !lowBatteryAlertSent
    )
    {
      String lowBatteryMessage =
        "SENTROVA LOW BATTERY ALERT\n\n";

      lowBatteryMessage +=
        "Battery: ";

      lowBatteryMessage +=
        String(
          batteryPercent
        );

      lowBatteryMessage +=
        "% (";

      lowBatteryMessage +=
        String(
          batteryVoltage,
          2
        );

      lowBatteryMessage +=
        " V)";

      sendTelegramMessage(
        lowBatteryMessage
      );

      lowBatteryAlertSent = true;
    }

    if (!batteryLow)
    {
      lowBatteryAlertSent =
        false;
    }
  }


  // ----------------------------------------------------------
  // BLE + AUDIO
  // ----------------------------------------------------------

  sendBLETElemetry();
  serviceSENTROVAAudio();


  // ----------------------------------------------------------
  // OLED
  // ----------------------------------------------------------

  if (
    now -
    lastDisplayUpdate >=
    200
  )
  {
    lastDisplayUpdate =
      now;

    handleOLEDTimeout();

    if (oledAwake)
      drawWatch();
  }


  // ----------------------------------------------------------
  // WEB SERVER
  // ----------------------------------------------------------

  server.handleClient();


  // ----------------------------------------------------------
  // SMALL YIELD
  // ----------------------------------------------------------

  delay(1);
}