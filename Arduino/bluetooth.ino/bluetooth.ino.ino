#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// =====================
// ===== TFT =====
// =====================
#define TFT_CS   D1
#define TFT_DC   D3
#define TFT_RST  D2

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// =====================
// ===== SD =====
// =====================
#define SD_CS    D0
#define SD_SCK   D8
#define SD_MISO  D9
#define SD_MOSI  D10

#define BTN_START  D5
#define BTN_STOP   D6
#define BTN_NEW    D7

File logFile;
bool logging = false;
String currentFilename = "";
uint32_t lastFlush = 0;

// =====================
// ===== BLE =====
// =====================
#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define LOG_CHAR_UUID       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

uint32_t g_lastRxMs = 0;
String   g_lastLine = "";

// 表示用パラメータ
uint32_t g_seq = 0;
uint8_t  g_phase = 0;
uint16_t g_vbat = 0;
float g_yaw=0, g_pitch=0, g_roll=0;
float g_lat=0, g_lon=0, g_alt=0;

// =====================
// ===== BLE Callbacks =====
// =====================
class LogCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {

    String s = c->getValue();
    g_lastLine = s;
    g_lastRxMs = millis();

    parseTelemetry(s);

    if (logging && logFile) {
      logFile.print(s);  // そのまま保存（改行含む）
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    Serial.println("BLE connected");
  }
  void onDisconnect(BLEServer* s) override {
    Serial.println("BLE disconnected -> restart advertising");
    BLEDevice::startAdvertising();
  }
};

// =====================
// ===== 文字列パース =====
// =====================
float getFloat(String s, String key) {
  int idx = s.indexOf(key);
  if (idx < 0) return NAN;
  int start = idx + key.length();
  int end = s.indexOf(",", start);
  if (end < 0) end = s.length();
  return s.substring(start, end).toFloat();
}

uint32_t getUInt(String s, String key) {
  int idx = s.indexOf(key);
  if (idx < 0) return 0;
  int start = idx + key.length();
  int end = s.indexOf(",", start);
  if (end < 0) end = s.length();
  return s.substring(start, end).toInt();
}

void parseTelemetry(String s) {
  g_seq   = getUInt(s, "SEQ=");
  g_phase = getUInt(s, "PH=");
  g_vbat  = getUInt(s, "VBAT=");

  g_yaw   = getFloat(s, "Y=");
  g_pitch = getFloat(s, "P=");
  g_roll  = getFloat(s, "R=");

  g_lat = getFloat(s, "LAT=");
  g_lon = getFloat(s, "LON=");
  g_alt = getFloat(s, "ALT=");
}

// =====================
// ===== 画面表示 =====
// =====================
void drawHeader() {
  tft.fillRect(0, 0, 240, 20, ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);

  float age = (millis() - g_lastRxMs) / 1000.0;

  tft.printf("VBAT:%4.2fV PH:%d age:%4.1fs",
             g_vbat/1000.0, g_phase, age);
}

void drawDebug() {
  tft.fillRect(0, 20, 240, 220, ST77XX_BLACK);
  tft.setCursor(0, 25);
  tft.setTextSize(2);
  tft.printf("SEQ:%lu\n", g_seq);
  tft.printf("Y:%.1f\n", g_yaw);
  tft.printf("P:%.1f\n", g_pitch);
  tft.printf("R:%.1f\n", g_roll);
  tft.printf("ALT:%.1f\n", g_alt);
}

// =====================
// ===== SD制御 =====
// =====================
void createNewFile() {

  if (logging) {
    logFile.close();
    logging = false;
  }

  int index = 1;
  char filename[20];

  while (true) {
    sprintf(filename, "/log_%03d.csv", index);
    if (!SD.exists(filename)) break;
    index++;
  }

  currentFilename = String(filename);

  File f = SD.open(currentFilename, FILE_WRITE);
  if (f) {
    f.println("SEQ,PH,VBAT,Y,P,R,LAT,LON,ALT");
    f.close();
    Serial.println("Created: " + currentFilename);
  }
}

void startLogging() {
  if (logging) return;
  if (currentFilename == "") createNewFile();

  logFile = SD.open(currentFilename, FILE_APPEND);
  if (!logFile) return;

  logging = true;
  Serial.println("Logging started");
}

void stopLogging() {
  if (!logging) return;

  logFile.flush();
  logFile.close();
  logging = false;

  Serial.println("Logging stopped");
}

// =====================
// ===== ボタン =====
// =====================
void handleButtons() {
  static bool lastS=0,lastP=0,lastN=0;

  bool s=digitalRead(BTN_START);
  bool p=digitalRead(BTN_STOP);
  bool n=digitalRead(BTN_NEW);

  if(!lastS && s) startLogging();
  if(!lastP && p) stopLogging();
  if(!lastN && n) createNewFile();

  lastS=s;
  lastP=p;
  lastN=n;
}

// =====================
// ===== setup =====
// =====================
void setup() {

  Serial.begin(115200);

  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, SPI, 4000000)) {
    Serial.println("SD init failed");
  }

  pinMode(BTN_START, INPUT);
  pinMode(BTN_STOP,  INPUT);
  pinMode(BTN_NEW,   INPUT);

  tft.init(240,240);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  BLEDevice::init("CanSat-Remote");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);
  BLECharacteristic* ch = service->createCharacteristic(
      LOG_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE
  );

  ch->setCallbacks(new LogCallbacks());
  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->start();

  Serial.println("Advertising started");
}

// =====================
// ===== loop =====
// =====================
void loop() {

  handleButtons();

  if (logging && millis() - lastFlush > 1000) {
    logFile.flush();
    lastFlush = millis();
  }

  drawHeader();
  drawDebug();

  delay(100);
}