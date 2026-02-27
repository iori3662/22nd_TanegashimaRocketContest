#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// ====== TFT pins (あなたのGPIOに合わせて変更) ======
#define TFT_CS D1
#define TFT_DC D3
#define TFT_RST D2

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

static const int W = 240, H = 240;
static const int HEADER_H = 24;

// ====== BLE UUID ======
static BLEUUID SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static BLEUUID LOG_UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");

// ====== RX data ======
volatile uint32_t g_lastRxMs = 0;
volatile uint32_t g_seq = 0;
volatile uint16_t g_vbat_mV = 7400;
volatile uint8_t g_phase = 0;
volatile uint16_t g_len = 0;

volatile float g_yaw = 0, g_pitch = 0, g_roll = 0;
volatile float g_lat = 0, g_lon = 0, g_alt = 0;
volatile uint16_t g_pwml = 0, g_pwmr = 0;

const char* phaseNames[] = { "INIT", "TEST", "ASCENT", "DESCENT", "RUN", "GOAL", "FAIL" };
static inline const char* phaseName(uint8_t p) {
  if (p < 7) return phaseNames[p];
  return "UNK";
}

// ====== UI helpers ======
void drawHeader() {
  float vbat = g_vbat_mV / 1000.0f;
  bool link_ok = (millis() - g_lastRxMs) < 3000;

  tft.fillRect(0, 0, W, HEADER_H, ST77XX_BLACK);
  tft.drawFastHLine(0, HEADER_H - 1, W, ST77XX_WHITE);

  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  tft.setCursor(4, 6);
  tft.print("VBAT:");
  tft.print(vbat, 2);
  tft.print("V");

  tft.setCursor(108, 6);
  tft.print("PH:");
  tft.print(phaseName(g_phase));

  tft.setCursor(190, 6);
  tft.print(link_ok ? "BLE" : "NO");
}

void drawDebugStatic() {
  tft.fillScreen(ST77XX_BLACK);
  drawHeader();

  tft.setTextWrap(false);
  tft.setTextSize(1);

  int y = HEADER_H + 8;
  tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
  tft.setCursor(6, y);
  tft.print("[DEBUG] RX from Pi");
  y += 16;

  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(6, y);
  tft.print("SEQ:");
  y += 14;
  tft.setCursor(6, y);
  tft.print("LEN:");
  y += 14;
  tft.setCursor(6, y);
  tft.print("AGE:");
  y += 14;
  tft.setCursor(6, y);
  tft.print("YPR:");
  y += 14;
  tft.setCursor(6, y);
  tft.print("LAT:");
  y += 14;
  tft.setCursor(6, y);
  tft.print("LON:");
  y += 14;
  tft.setCursor(6, y);
  tft.print("ALT:");
  y += 14;
  tft.setCursor(6, y);
  tft.print("PWM:");
  y += 14;
}

static void clearValueLine(int y) {
  tft.fillRect(60, y, 175, 12, ST77XX_BLACK);
}

void updateDebugValues() {
  Serial.println("UI tick");
  drawHeader();

  tft.setTextWrap(false);
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  uint32_t age = millis() - g_lastRxMs;

  int y = HEADER_H + 8 + 16;

  clearValueLine(y);
  tft.setCursor(60, y);
  tft.print(g_seq);
  y += 14;

  clearValueLine(y);
  tft.setCursor(60, y);
  tft.print(g_len);
  tft.print(" bytes");
  y += 14;

  clearValueLine(y);
  tft.setCursor(60, y);
  tft.print(age);
  tft.print(" ms");
  y += 14;

  clearValueLine(y);
  tft.setCursor(60, y);
  tft.print(g_yaw, 1);
  tft.print(",");
  tft.print(g_pitch, 1);
  tft.print(",");
  tft.print(g_roll, 1);
  y += 14;

  clearValueLine(y);
  tft.setCursor(60, y);
  tft.print(g_lat, 6);
  y += 14;

  clearValueLine(y);
  tft.setCursor(60, y);
  tft.print(g_lon, 6);
  y += 14;

  clearValueLine(y);
  tft.setCursor(60, y);
  tft.print(g_alt, 1);
  tft.print(" m");
  y += 14;

  clearValueLine(y);
  tft.setCursor(60, y);
  tft.print(g_pwml);
  tft.print("/");
  tft.print(g_pwmr);
  y += 14;
}

// ====== ASCII parsing ======
static bool parseIntField(const String& s, const char* key, uint32_t& out) {
  int i = s.indexOf(key);
  if (i < 0) return false;
  i += strlen(key);
  int j = s.indexOf(',', i);
  String v = (j < 0) ? s.substring(i) : s.substring(i, j);
  out = (uint32_t)v.toInt();
  return true;
}

static bool parseFloatField(const String& s, const char* key, float& out) {
  int i = s.indexOf(key);
  if (i < 0) return false;
  i += strlen(key);
  int j = s.indexOf(',', i);
  String v = (j < 0) ? s.substring(i) : s.substring(i, j);
  out = v.toFloat();
  return true;
}

class LogCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String s = c->getValue();

    Serial.print("RX len=");
    Serial.println((int)s.length());
    Serial.println(s);  // 最初だけでもOK（重いならコメントアウト）

    g_len = (uint16_t)s.length();
    g_lastRxMs = millis();

    uint32_t seq = g_seq, ph = g_phase, vbat = g_vbat_mV;
    parseIntField(s, "SEQ=", seq);
    parseIntField(s, "PH=", ph);
    parseIntField(s, "VBAT=", vbat);

    g_seq = seq;
    g_phase = (uint8_t)ph;
    g_vbat_mV = (uint16_t)vbat;

    float f;
    if (parseFloatField(s, "Y=", f)) g_yaw = f;
    if (parseFloatField(s, "P=", f)) g_pitch = f;
    if (parseFloatField(s, "R=", f)) g_roll = f;
    if (parseFloatField(s, "LAT=", f)) g_lat = f;
    if (parseFloatField(s, "LON=", f)) g_lon = f;
    if (parseFloatField(s, "ALT=", f)) g_alt = f;

    uint32_t u;
    if (parseIntField(s, "PWML=", u)) g_pwml = (uint16_t)u;
    if (parseIntField(s, "PWMR=", u)) g_pwmr = (uint16_t)u;
  }
};

// （任意）接続状態を見たい場合のサーバコールバック
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    Serial.println("BLE connected");
  }
  void onDisconnect(BLEServer* s) override {
    Serial.println("BLE disconnected -> restart advertising");
    BLEDevice::startAdvertising();  // 再広告（重要：再実行時の取りこぼし減）
  }
};

void setup() {
  Serial.begin(115200);
  delay(200);

  SPI.begin();
  tft.init(W, H);
  tft.setSPISpeed(40000000);
  tft.setRotation(1);

  drawDebugStatic();

  BLEDevice::init("CanSat-Remote");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  BLEService* svc = server->createService(SERVICE_UUID);

  BLECharacteristic* logChar = svc->createCharacteristic(
    LOG_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  logChar->setCallbacks(new LogCallbacks());

  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->start();

  Serial.println("Advertising started: CanSat-Remote");
}

void loop() {
  static uint32_t lastUi = 0;
  uint32_t now = millis();
  if (now - lastUi > 200) {  // 5Hz
    lastUi = now;
    updateDebugValues();
  }
  delay(5);
}