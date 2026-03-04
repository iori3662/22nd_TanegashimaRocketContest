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
// Pin config (your requested)
// =====================
#define TFT_CS   D1
#define TFT_DC   D3
#define TFT_RST  D2

#define SD_CS    D0
#define SD_SCK   D8
#define SD_MISO  D9
#define SD_MOSI  D10

#define BTN_L    D5
#define BTN_OK   D6
#define BTN_R    D7

#define BUZZER   D4

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// =====================
// BLE Nordic UART UUIDs
// =====================
#define UART_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define UART_RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // Pi->XIAO (Write)
#define UART_TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // XIAO->Pi (Notify)

BLECharacteristic* g_tx = nullptr;
volatile bool g_bleConnected = false;

// =====================
// Goal coordinates (EDIT THIS)
// =====================
static const double GOAL_LAT = 35.000000;   // <-- set
static const double GOAL_LON = 135.000000;  // <-- set

// =====================
// CSV (VBAT removed)
// 16 columns fixed:
// seq,ph,yaw,pitch,roll,lat,lon,alt_gps,temp,press,hum,alt_bme,ax,ay,az,a
// =====================
static const char* CSV_HEADER =
"seq,ph,yaw_deg,pitch_deg,roll_deg,lat_deg,lon_deg,alt_gps_m,temp_C,press_hPa,hum_pct,alt_bme_m,ax,ay,az,a";
static const int CSV_COLS = 16;

// =====================
// Logging
// =====================
File g_logFile;
bool g_logging = false;
String g_filename = "";
uint32_t g_lastFlushMs = 0;

// =====================
// Telemetry
// =====================
uint32_t g_lastRxMs = 0;

uint32_t g_seq = 0;
uint8_t  g_ph = 0;

float g_yaw=NAN, g_pitch=NAN, g_roll=NAN;
float g_lat=NAN, g_lon=NAN, g_alt_gps=NAN;
float g_temp=NAN, g_press=NAN, g_hum=NAN;
float g_altb=NAN;
float g_ax=NAN, g_ay=NAN, g_az=NAN, g_a=NAN;

// =====================
// Comms metrics
// =====================
uint32_t g_lastSeq = 0;
bool     g_haveSeq = false;
uint32_t g_rxCount = 0;
uint32_t g_lostCount = 0;
uint32_t g_seqGapLast = 0;

uint32_t g_hzWindowStartMs = 0;
uint32_t g_hzWindowRx = 0;
float    g_rxHz = 0.0f;  // measured

// =====================
// Altitude graph buffer
// =====================
static const int ALT_BUF_N = 120;        // ~ 120 points
float g_altBuf[ALT_BUF_N];
int   g_altHead = 0;
int   g_altCount = 0;

// =====================
// UI
// =====================
enum ScreenMode { SCR_HOME, SCR_DEBUG, SCR_RUN, SCR_STATUS };
ScreenMode g_mode = SCR_HOME;

int g_menuIndex = 0;
const char* MENU_ITEMS[] = {
  "DEBUG",
  "RUN",
  "STATUS",
  "PHASE +1",
  "PHASE -1",
  "LOG TOGGLE",
  "NEW FILE"
};
const int MENU_N = (int)(sizeof(MENU_ITEMS)/sizeof(MENU_ITEMS[0]));

// redraw control
volatile bool g_dirtyTelemetry = false;  // set when RX comes
bool g_dirtyUI = true;                  // set when menu/screen changes
bool g_staticDrawn = false;             // per-screen static layout drawn

uint32_t g_lastDrawMs = 0;
const uint32_t DRAW_PERIOD_MS = 200;    // 5Hz max
uint32_t g_lastGraphMs = 0;
const uint32_t GRAPH_PERIOD_MS = 500;   // 2Hz graph redraw (enough)

// =====================
// Buttons (short/long)
// =====================
const uint32_t LONG_MS = 800;
struct BtnState { bool last=false; uint32_t downMs=0; bool longFired=false; };
BtnState bL, bOK, bR;

// =====================
// Utilities
// =====================
static void beep() {
  tone(BUZZER, 2000, 40);
}

static double deg2rad(double d){ return d * 3.14159265358979323846 / 180.0; }

static double haversine_m(double lat1, double lon1, double lat2, double lon2){
  // If lat/lon invalid -> NaN
  if (isnan(lat1) || isnan(lon1) || isnan(lat2) || isnan(lon2)) return NAN;

  double R = 6371000.0;
  double p1 = deg2rad(lat1);
  double p2 = deg2rad(lat2);
  double dphi = deg2rad(lat2 - lat1);
  double dl = deg2rad(lon2 - lon1);

  double a = sin(dphi/2)*sin(dphi/2) + cos(p1)*cos(p2)*sin(dl/2)*sin(dl/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

static float parseFloatToken(String tok){
  tok.trim(); tok.toLowerCase();
  if (tok == "nan" || tok.length()==0) return NAN;
  return tok.toFloat();
}
static uint32_t parseUIntToken(String tok){
  tok.trim();
  if (tok.length()==0) return 0;
  return (uint32_t)tok.toInt();
}

static bool splitCsvN(const String& s, String* outTok, int n){
  int start=0, col=0;
  while (col < n) {
    int comma = s.indexOf(',', start);
    if (comma < 0) { outTok[col++] = s.substring(start); break; }
    outTok[col++] = s.substring(start, comma);
    start = comma + 1;
  }
  return (col >= n);
}

static void pushAltPoint(float altb){
  if (isnan(altb)) return;
  g_altBuf[g_altHead] = altb;
  g_altHead = (g_altHead + 1) % ALT_BUF_N;
  if (g_altCount < ALT_BUF_N) g_altCount++;
}

static void sendCmd(const String& name, const String& arg){
  if (!g_tx || !g_bleConnected) return;
  String msg = "cmd," + name + "," + arg + "\n";
  g_tx->setValue(msg.c_str());
  g_tx->notify();
  Serial.print("TX: "); Serial.print(msg);
}

// =====================
// SD logging
// =====================
static void stopLogging(){
  if (!g_logging) return;
  g_logFile.flush();
  g_logFile.close();
  g_logging = false;
  Serial.println("Logging stopped");
  g_dirtyUI = true;
}
static void createNewFile(){
  if (g_logging) stopLogging();

  int idx=1;
  char fn[24];
  while (true){
    sprintf(fn, "/log_%03d.csv", idx);
    if (!SD.exists(fn)) break;
    idx++;
  }
  g_filename = String(fn);

  File f = SD.open(g_filename, FILE_WRITE);
  if (f){
    f.println(CSV_HEADER);
    f.close();
    Serial.print("Created: "); Serial.println(g_filename);
  } else {
    Serial.println("File creation failed");
  }
  g_dirtyUI = true;
}
static void startLogging(){
  if (g_logging) return;
  if (g_filename == "") createNewFile();

  g_logFile = SD.open(g_filename, FILE_APPEND);
  if (!g_logFile){
    Serial.println("File open failed");
    return;
  }
  g_logging = true;
  Serial.println("Logging started");
  g_dirtyUI = true;
}

// =====================
// Drawing helpers
// =====================
static void drawHeader(){
  // small band only
  tft.fillRect(0, 0, 240, 26, ST77XX_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(2);

  float age = (g_lastRxMs==0) ? NAN : (millis() - g_lastRxMs)/1000.0f;
  if (isnan(age)) {
    tft.printf("PH:%u  age:--.-", g_ph);
  } else {
    tft.printf("PH:%u  age:%4.1f", g_ph, age);
  }

  // comm indicator
  tft.setCursor(0, 18);
  tft.setTextSize(1);
  tft.printf("%s  Hz:%.1f  loss:%lu  gap:%lu",
             g_bleConnected ? "BLE:OK" : "BLE:--",
             g_rxHz,
             (unsigned long)g_lostCount,
             (unsigned long)g_seqGapLast);
}

static void drawFooter(){
  tft.fillRect(0, 220, 240, 20, ST77XX_BLACK);
  tft.setCursor(0, 222);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.printf("LOG:%s  %s", g_logging ? "ON" : "OFF", g_filename.c_str());
}

static void clearBody(){
  tft.fillRect(0, 26, 240, 194, ST77XX_BLACK);
}

// =====================
// Screens: static layout + update
// =====================
static void homeStatic(){
  clearBody();
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  tft.setTextSize(3);
  tft.setCursor(0, 32);
  tft.println("HOME");

  tft.setTextSize(2);
  tft.setCursor(0, 68);
  tft.println("L/R:Sel  OK:Go");

  // menu frame (we will redraw only selection lines)
  tft.setTextSize(2);
  for (int i=0;i<MENU_N;i++){
    tft.setCursor(0, 96 + i*18);
    tft.println("                "); // placeholder line
  }
}

static void homeUpdateMenu(){
  // only rewrite menu lines (small region)
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(2);
  for (int i=0;i<MENU_N;i++){
    tft.fillRect(0, 96 + i*18, 240, 18, ST77XX_BLACK);
    tft.setCursor(0, 96 + i*18);
    if (i == g_menuIndex) tft.print("> ");
    else tft.print("  ");
    tft.println(MENU_ITEMS[i]);
  }
}

static void debugStatic(){
  clearBody();
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  tft.setTextSize(2);
  tft.setCursor(0, 30);
  tft.println("DEBUG");

  tft.setTextSize(2);
  tft.setCursor(0, 56); tft.print("SEQ:");
  tft.setCursor(0, 76); tft.print("YPR:");
  tft.setCursor(0, 96); tft.print("ALTb:");
  tft.setCursor(0, 116); tft.print("A:");
  tft.setCursor(0, 136); tft.print("GPS:");
  tft.setCursor(0, 156); tft.print("DST:");

  tft.setTextSize(1);
  tft.setCursor(0, 196); tft.println("OK:HOME");
}

static void debugUpdate(){
  // update value fields only (no full screen clear)
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  // SEQ
  tft.fillRect(70, 56, 170, 18, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(70, 56);
  tft.printf("%lu", (unsigned long)g_seq);

  // YPR
  tft.fillRect(70, 76, 170, 18, ST77XX_BLACK);
  tft.setCursor(70, 76);
  tft.printf("%.0f %.0f %.0f", g_yaw, g_pitch, g_roll);

  // ALTb
  tft.fillRect(70, 96, 170, 18, ST77XX_BLACK);
  tft.setCursor(70, 96);
  tft.printf("%.2f", g_altb);

  // A
  tft.fillRect(70, 116, 170, 18, ST77XX_BLACK);
  tft.setCursor(70, 116);
  tft.printf("%.2f", g_a);

  // GPS (lat lon)
  tft.fillRect(70, 136, 170, 18, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(70, 136);
  tft.printf("%.5f %.5f", g_lat, g_lon);

  // Distance
  double d = haversine_m(g_lat, g_lon, GOAL_LAT, GOAL_LON);
  tft.fillRect(70, 156, 170, 18, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(70, 156);
  if (isnan(d)) tft.print("--");
  else tft.printf("%.0fm", d);
}

static void runStatic(){
  clearBody();
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  tft.setTextSize(3);
  tft.setCursor(0, 60);
  tft.println("RUN");

  tft.setTextSize(2);
  tft.setCursor(0, 110);
  tft.print("PH=");

  tft.setTextSize(1);
  tft.setCursor(0, 196); tft.println("OK:HOME");
}

static void runUpdate(){
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(2);
  tft.fillRect(50, 110, 80, 20, ST77XX_BLACK);
  tft.setCursor(50, 110);
  tft.printf("%u", g_ph);
}

static void statusStatic(){
  clearBody();
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  tft.setTextSize(2);
  tft.setCursor(0, 30);
  tft.println("STATUS");

  // graph frame
  tft.drawRect(0, 52, 240, 90, ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(0, 144);
  tft.println("ALT graph (BME)");

  // info lines
  tft.setTextSize(2);
  tft.setCursor(0, 160); tft.print("DST:");
  tft.setCursor(0, 180); tft.print("RX :");

  tft.setTextSize(1);
  tft.setCursor(0, 196); tft.println("OK:HOME");
}

static void statusUpdateInfo(){
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);

  double d = haversine_m(g_lat, g_lon, GOAL_LAT, GOAL_LON);

  tft.setTextSize(2);
  tft.fillRect(60, 160, 180, 18, ST77XX_BLACK);
  tft.setCursor(60, 160);
  if (isnan(d)) tft.print("--");
  else tft.printf("%.0fm", d);

  float age = (g_lastRxMs==0) ? NAN : (millis() - g_lastRxMs)/1000.0f;

  tft.fillRect(60, 180, 180, 18, ST77XX_BLACK);
  tft.setCursor(60, 180);
  if (isnan(age)) {
    tft.printf("--.-s");
  } else {
    tft.printf("%.1fs", age);
  }
}

static void statusDrawGraph(){
  // Redraw only inside graph box (0,52)-(240,142)
  const int gx=1, gy=53, gw=238, gh=88;

  // clear plot area
  tft.fillRect(gx, gy, gw, gh, ST77XX_BLACK);

  if (g_altCount < 2) return;

  // find min/max in buffer
  float amin=1e9f, amax=-1e9f;
  for (int i=0;i<g_altCount;i++){
    int idx = (g_altHead - g_altCount + i);
    while (idx < 0) idx += ALT_BUF_N;
    idx %= ALT_BUF_N;
    float v = g_altBuf[idx];
    if (!isnan(v)) {
      if (v < amin) amin = v;
      if (v > amax) amax = v;
    }
  }
  if (amax <= amin) { amax = amin + 0.1f; }

  // polyline
  int prevx=-1, prevy=-1;
  for (int i=0;i<g_altCount;i++){
    int idx = (g_altHead - g_altCount + i);
    while (idx < 0) idx += ALT_BUF_N;
    idx %= ALT_BUF_N;
    float v = g_altBuf[idx];
    if (isnan(v)) continue;

    int x = gx + (int)((long long)i * (gw-1) / (g_altCount-1));
    float t = (v - amin) / (amax - amin);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    int y = gy + (gh-1) - (int)(t * (gh-1));

    if (prevx >= 0) {
      tft.drawLine(prevx, prevy, x, y, ST77XX_WHITE);
    }
    prevx = x; prevy = y;
  }

  // min/max labels (small)
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(2, 54);
  tft.printf("%.1f", amax);
  tft.setCursor(2, 53+gh-10);
  tft.printf("%.1f", amin);
}

// =====================
// Buttons (short/long)
// =====================
static void handleBtnOne(bool cur, BtnState& bs, void (*onShort)(), void (*onLong)()){
  uint32_t now = millis();
  if (!bs.last && cur) { bs.downMs = now; bs.longFired=false; }
  if (cur && !bs.longFired && (now - bs.downMs >= LONG_MS)) { bs.longFired=true; if(onLong) onLong(); }
  if (bs.last && !cur) { if(!bs.longFired) if(onShort) onShort(); }
  bs.last = cur;
}

static void onL_short(){
  beep();
  if (g_mode == SCR_HOME){
    g_menuIndex = (g_menuIndex - 1 + MENU_N) % MENU_N;
    g_dirtyUI = true;
  }
}
static void onR_short(){
  beep();
  if (g_mode == SCR_HOME){
    g_menuIndex = (g_menuIndex + 1) % MENU_N;
    g_dirtyUI = true;
  }
}
static void onOK_short(){
  beep();
  if (g_mode != SCR_HOME){
    g_mode = SCR_HOME;
    g_dirtyUI = true;
    g_staticDrawn = false;
    return;
  }

  const char* item = MENU_ITEMS[g_menuIndex];

  if (strcmp(item, "DEBUG")==0) {
    g_mode = SCR_DEBUG;
    sendCmd("mode","debug");
    g_dirtyUI = true;
    g_staticDrawn = false;
  } else if (strcmp(item, "RUN")==0) {
    g_mode = SCR_RUN;
    sendCmd("mode","run");
    g_dirtyUI = true;
    g_staticDrawn = false;
  } else if (strcmp(item, "STATUS")==0) {
    g_mode = SCR_STATUS;
    g_dirtyUI = true;
    g_staticDrawn = false;
  } else if (strcmp(item, "PHASE +1")==0) {
    sendCmd("phase","inc");
  } else if (strcmp(item, "PHASE -1")==0) {
    sendCmd("phase","dec");
  } else if (strcmp(item, "LOG TOGGLE")==0) {
    if (g_logging) stopLogging(); else startLogging();
    sendCmd("log","toggle");
  } else if (strcmp(item, "NEW FILE")==0) {
    createNewFile();
  }
}

static void onL_long(){
  beep();
  sendCmd("mode","idle");
  g_mode = SCR_HOME;
  g_dirtyUI = true;
  g_staticDrawn = false;
}
static void onOK_long(){
  beep();
  if (g_logging) stopLogging(); else startLogging();
}
static void onR_long(){
  beep();
  createNewFile();
}

// =====================
// BLE callbacks
// =====================
class RxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String s = c->getValue();
    s.trim();
    if (s.length()==0) return;
    if (s.startsWith("seq,ph,")) return; // ignore header

    // Serial: keep it (but can be heavy; still requested)
    Serial.print("RX: "); Serial.println(s);

    // parse
    String tok[CSV_COLS];
    if (!splitCsvN(s, tok, CSV_COLS)) return;

    uint32_t seq = parseUIntToken(tok[0]);
    uint8_t  ph  = (uint8_t)parseUIntToken(tok[1]);

    // comm metrics
    g_rxCount++;
    if (!g_haveSeq) {
      g_haveSeq = true;
      g_lastSeq = seq;
      g_seqGapLast = 0;
      g_hzWindowStartMs = millis();
      g_hzWindowRx = 0;
    } else {
      if (seq > g_lastSeq + 1) {
        uint32_t gap = seq - g_lastSeq - 1;
        g_lostCount += gap;
        g_seqGapLast = gap;
      } else {
        g_seqGapLast = 0;
      }
      g_lastSeq = seq;
    }

    // measured rx Hz (window 2s)
    g_hzWindowRx++;
    uint32_t now = millis();
    uint32_t dt = now - g_hzWindowStartMs;
    if (dt >= 2000) {
      g_rxHz = (dt > 0) ? (1000.0f * (float)g_hzWindowRx / (float)dt) : 0.0f;
      g_hzWindowStartMs = now;
      g_hzWindowRx = 0;
    }

    // assign fields
    g_seq = seq;
    g_ph  = ph;

    g_yaw   = parseFloatToken(tok[2]);
    g_pitch = parseFloatToken(tok[3]);
    g_roll  = parseFloatToken(tok[4]);

    g_lat   = parseFloatToken(tok[5]);
    g_lon   = parseFloatToken(tok[6]);
    g_alt_gps = parseFloatToken(tok[7]);

    g_temp  = parseFloatToken(tok[8]);
    g_press = parseFloatToken(tok[9]);
    g_hum   = parseFloatToken(tok[10]);

    g_altb  = parseFloatToken(tok[11]);
    g_ax    = parseFloatToken(tok[12]);
    g_ay    = parseFloatToken(tok[13]);
    g_az    = parseFloatToken(tok[14]);
    g_a     = parseFloatToken(tok[15]);

    // altitude graph
    pushAltPoint(g_altb);

    g_lastRxMs = now;
    g_dirtyTelemetry = true;

    // SD: value-only CSV
    if (g_logging && g_logFile) {
      g_logFile.println(s);
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    g_bleConnected = true;
    Serial.println("BLE connected");
    g_dirtyUI = true;
  }
  void onDisconnect(BLEServer*) override {
    g_bleConnected = false;
    Serial.println("BLE disconnected -> restart advertising");
    BLEDevice::startAdvertising();
    g_dirtyUI = true;
  }
};

// =====================
// setup/loop
// =====================
void setup(){
  Serial.begin(115200);
  delay(200);

  pinMode(BTN_L, INPUT);
  pinMode(BTN_OK, INPUT);
  pinMode(BTN_R, INPUT);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

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
    createNewFile();
    startLogging();
  }

  // BLE
  BLEDevice::init("CanSat-Remote");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService* svc = server->createService(UART_SERVICE_UUID);

  BLECharacteristic* rx = svc->createCharacteristic(
    UART_RX_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  rx->setCallbacks(new RxCallbacks());

  g_tx = svc->createCharacteristic(
    UART_TX_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  g_tx->addDescriptor(new BLE2902());

  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(UART_SERVICE_UUID);
  adv->start();

  Serial.println("Advertising started: CanSat-Remote");

  g_dirtyUI = true;
  g_staticDrawn = false;
}

static void drawScreenStaticIfNeeded(){
  if (g_staticDrawn) return;

  // keep header/footer persistent; body depends on screen
  clearBody();

  if (g_mode == SCR_HOME) {
    homeStatic();
    homeUpdateMenu();
  } else if (g_mode == SCR_DEBUG) {
    debugStatic();
    debugUpdate();
  } else if (g_mode == SCR_RUN) {
    runStatic();
    runUpdate();
  } else if (g_mode == SCR_STATUS) {
    statusStatic();
    statusUpdateInfo();
    statusDrawGraph();
  }

  g_staticDrawn = true;
}

void loop(){
  // buttons
  bool l = digitalRead(BTN_L);
  bool o = digitalRead(BTN_OK);
  bool r = digitalRead(BTN_R);
  handleBtnOne(l, bL, onL_short, onL_long);
  handleBtnOne(o, bOK, onOK_short, onOK_long);
  handleBtnOne(r, bR, onR_short, onR_long);

  // SD flush
  if (g_logging && g_logFile && (millis() - g_lastFlushMs > 1000)) {
    g_logFile.flush();
    g_lastFlushMs = millis();
  }

  uint32_t now = millis();

  // throttle draw
  if (now - g_lastDrawMs >= DRAW_PERIOD_MS) {
    // static layout if needed
    if (g_dirtyUI) {
      g_staticDrawn = false;
      g_dirtyUI = false;
    }
    drawScreenStaticIfNeeded();

    // header/footer always (small areas)
    drawHeader();
    drawFooter();

    // update dynamic areas depending on screen
    bool tele = g_dirtyTelemetry;
    if (tele) {
      if (g_mode == SCR_HOME) {
        // HOME: only menu changes; telemetry doesn't require body updates
      } else if (g_mode == SCR_DEBUG) {
        debugUpdate();
      } else if (g_mode == SCR_RUN) {
        runUpdate();
      } else if (g_mode == SCR_STATUS) {
        statusUpdateInfo();
      }
      g_dirtyTelemetry = false;
    }

    // graph redraw at slower rate on STATUS
    if (g_mode == SCR_STATUS && (now - g_lastGraphMs >= GRAPH_PERIOD_MS)) {
      statusDrawGraph();
      g_lastGraphMs = now;
    }

    g_lastDrawMs = now;
  }

  // Give time to BLE task (important)
  delay(10);
}