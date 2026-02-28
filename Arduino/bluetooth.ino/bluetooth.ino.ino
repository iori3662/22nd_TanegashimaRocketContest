#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

// =====================
// TFT pins（必要なら変更）
// =====================
#define TFT_CS   D1
#define TFT_DC   D3
#define TFT_RST  D2

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// =====================
// SD pins（※TFT_CSと衝突させない）
// =====================
#define SD_CS    D0     // ★ D0は使わない（衝突回避）
#define SD_SCK   D8
#define SD_MISO  D9
#define SD_MOSI  D10

// =====================
// Buttons
// =====================
#define BTN_START  D5
#define BTN_STOP   D6
#define BTN_NEW    D7

// =====================
// BLE UUID
// =====================
#define SERVICE_UUID   "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define LOG_CHAR_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

// ====== Forward declarations（★これが無いと今回のエラーになる） ======
void parseTelemetry(const String& s);
void startLogging();
void stopLogging();
void createNewFile();
void handleButtons();
void drawHeader();
void drawDebug();

// =====================
// Globals
// =====================
File logFile;
bool logging = false;
String currentFilename = "";
uint32_t lastFlush = 0;

uint32_t g_lastRxMs = 0;
String   g_lastLine = "";

uint32_t g_seq = 0;
uint8_t  g_phase = 0;
uint16_t g_vbat_mV = 0;
float g_yaw=0, g_pitch=0, g_roll=0;
float g_lat=0, g_lon=0, g_alt=0;

// ---------------------
// Telemetry parse helpers
// ---------------------
static float getFloatField(const String& s, const char* key) {
  int i = s.indexOf(key);
  if (i < 0) return NAN;
  i += strlen(key);
  int j = s.indexOf(',', i);
  if (j < 0) j = s.length();
  return s.substring(i, j).toFloat();
}

static uint32_t getUIntField(const String& s, const char* key) {
  int i = s.indexOf(key);
  if (i < 0) return 0;
  i += strlen(key);
  int j = s.indexOf(',', i);
  if (j < 0) j = s.length();
  return (uint32_t)s.substring(i, j).toInt();
}

// =====================
// BLE Callbacks
// =====================
class LogCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String s = c->getValue();
    g_lastLine = s;
    g_lastRxMs = millis();

    parseTelemetry(s);

    // SDへそのまま保存（Pi側フォーマット=提出フォーマットにできる）
    if (logging && logFile) {
      logFile.print(s);  // 受信文字列に \n が含まれている前提
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    Serial.println("BLE connected");
  }
  void onDisconnect(BLEServer*) override {
    Serial.println("BLE disconnected -> restart advertising");
    BLEDevice::startAdvertising();
  }
};

// =====================
// Telemetry parser
// =====================
void parseTelemetry(const String& s) {
  g_seq      = getUIntField(s, "SEQ=");
  g_phase    = (uint8_t)getUIntField(s, "PH=");
  g_vbat_mV  = (uint16_t)getUIntField(s, "VBAT=");

  g_yaw   = getFloatField(s, "Y=");
  g_pitch = getFloatField(s, "P=");
  g_roll  = getFloatField(s, "R=");

  g_lat = getFloatField(s, "LAT=");
  g_lon = getFloatField(s, "LON=");
  g_alt = getFloatField(s, "ALT=");
}

// =====================
// UI
// =====================
void drawHeader() {
  tft.fillRect(0, 0, 240, 20, ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(1);

  float age = (millis() - g_lastRxMs) / 1000.0f;
  tft.printf("VBAT:%4.2fV PH:%u age:%4.1fs",
             g_vbat_mV / 1000.0f, g_phase, age);
}

void drawDebug() {
  tft.fillRect(0, 20, 240, 220, ST77XX_BLACK);
  tft.setCursor(0, 24);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(2);

  tft.printf("SEQ:%lu\n", (unsigned long)g_seq);
  tft.printf("Y:%.1f\n", g_yaw);
  tft.printf("P:%.1f\n", g_pitch);
  tft.printf("R:%.1f\n", g_roll);
  tft.printf("ALT:%.1f\n", g_alt);

  tft.setTextSize(1);
  tft.printf("LAT:%.6f\n", g_lat);
  tft.printf("LON:%.6f\n", g_lon);

  tft.setCursor(0, 210);
  tft.printf("LOG:%s  %s", logging ? "ON" : "OFF", currentFilename.c_str());
}

// =====================
// SD logging controls
// =====================
void startLogging() {
  if (logging) return;
  if (currentFilename == "") createNewFile();

  logFile = SD.open(currentFilename, FILE_APPEND);
  if (!logFile) {
    Serial.println("File open failed");
    return;
  }
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

void createNewFile() {
  if (logging) stopLogging();

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
    // 受信の実体は "SEQ=...,PH=...,..." なので、ヘッダは任意。
    // 「受信生ログ」と割り切るならヘッダ無しでも良い。
    f.println("raw_ascii_telemetry");
    f.close();
    Serial.print("Created: ");
    Serial.println(currentFilename);
  } else {
    Serial.println("File creation failed");
  }
}

// =====================
// Buttons
// =====================
void handleButtons() {
  static bool lastS = LOW, lastP = LOW, lastN = LOW;

  bool s = digitalRead(BTN_START);
  bool p = digitalRead(BTN_STOP);
  bool n = digitalRead(BTN_NEW);

  // START
  if (lastS == LOW && s == HIGH) {
    delay(20);
    if (digitalRead(BTN_START) == HIGH) startLogging();
  }
  // STOP
  if (lastP == LOW && p == HIGH) {
    delay(20);
    if (digitalRead(BTN_STOP) == HIGH) stopLogging();
  }
  // NEW
  if (lastN == LOW && n == HIGH) {
    delay(20);
    if (digitalRead(BTN_NEW) == HIGH) createNewFile();
  }

  lastS = s; lastP = p; lastN = n;
}

// =====================
// setup / loop
// =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(BTN_START, INPUT);
  pinMode(BTN_STOP,  INPUT);
  pinMode(BTN_NEW,   INPUT);

  // TFT
  tft.init(240, 240);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);

  // SD
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS, SPI, 4000000)) {
    Serial.println("SD init failed");
  } else {
    Serial.println("SD OK");
  }

  // BLE
  BLEDevice::init("CanSat-Remote");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* service = server->createService(SERVICE_UUID);

  BLECharacteristic* ch = service->createCharacteristic(
    LOG_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  ch->setCallbacks(new LogCallbacks());

  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->start();

  Serial.println("Advertising started: CanSat-Remote");
}

void loop() {
  handleButtons();

  if (logging && logFile && (millis() - lastFlush > 1000)) {
    logFile.flush();
    lastFlush = millis();
  }

  drawHeader();
  drawDebug();

  delay(100);
}