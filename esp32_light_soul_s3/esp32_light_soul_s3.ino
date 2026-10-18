/* Notes for myself
 * Light Soul — ESP32-S3 N16R8 Hardware Bridge
 * Full hardware: B10K slider · KY-023 joystick (+ SW) · KY-040 rotary
 *               Reed switches (zigzag 8 steps + 6 magnet snaps)
 *               3 mode buttons (SHAPE / FEEL / MUSIC) · 2 mode LEDs
 *
 * Connect to "LightSoul" WiFi, open http://192.168.4.1
 *
 * ── WIRING ──────────────────────────────────────────────────────────────────
 * B10K Linear Pot (straight slider)
 *   Left leg → GND   Wiper → GPIO1   Right leg → 3.3V
 *
 * KY-023 Joystick
 *   VCC→3.3V  GND→GND  VRX→GPIO2  VRY→GPIO4  SW→GPIO5
 *
 * KY-040 Rotary Encoder
 *   VCC→3.3V  GND→GND  CLK→GPIO6  DT→GPIO7
 *
 * Zigzag reed switches (8 steps, highest closed = current step)
 *   Step 1→GPIO8  Step 2→GPIO9  Step 3→GPIO10  Step 4→GPIO11
 *   Step 5→GPIO12  Step 6→GPIO13  Step 7→GPIO14  Step 8→GPIO15
 *
 * Magnet snap reed switches (6 positions, radio style)
 *   Snap 0→GPIO16  Snap 1→GPIO17  Snap 2→GPIO18
 *   Snap 3→GPIO21  Snap 4→GPIO38  Snap 5→GPIO39
 *
 * Buttons (active LOW, INPUT_PULLUP)
 *   SHAPE→GPIO40  FEEL→GPIO41  MUSIC→GPIO42
 *
 * LEDs (220Ω resistor to GND)
 *   SHAPE LED→GPIO47  FEEL LED→GPIO48
 *
 * ── AVOID ON N16R8 ──────────────────────────────────────────────────────────
 *   GPIO19-20 (USB)  GPIO26-37 (Octal PSRAM)
 *   GPIO43-44 (UART0)  GPIO0,3,45,46 (strapping)
 *
 * ── BOARD SETTINGS ──────────────────────────────────────────────────────────
 *   Board: ESP32S3 Dev Module
 *   PSRAM: OPI PSRAM   Flash: 16MB
 *   USB CDC On Boot: Enabled
 */

#include <WiFi.h>
#include <WebSocketsServer.h>

const char* AP_SSID     = "LightSoul";
const char* AP_PASSWORD = "lightsoul";

// ── Pins ─────────────────────────────────────────────────────────────────────
const int SLIDER_PIN = 1;
const int JOY_X_PIN  = 2;
const int JOY_Y_PIN  = 4;
const int JOY_SW_PIN = 5;
const int ROT_CLK    = 6;
const int ROT_DT     = 7;

const int ZIGZAG_COUNT = 8;
const int ZIGZAG_PINS[ZIGZAG_COUNT] = { 8, 9, 10, 11, 12, 13, 14, 15 };

const int SNAP_COUNT = 6;
const int SNAP_PINS[SNAP_COUNT] = { 16, 17, 18, 21, 38, 39 };

const int BTN_SHAPE = 40;
const int BTN_FEEL  = 41;
const int BTN_MUSIC = 42;
const int LED_SHAPE = 47;
const int LED_FEEL  = 48;

// ── Joystick center calibration ───────────────────────────────────────────────
// Watch Serial Monitor, DON'T touch the joystick, note the JX/JY values, set here.
int JX_CTR      = 1950;
int JY_CTR      = 1950;
const int JOY_DEADZONE = 150;

// ── State ────────────────────────────────────────────────────────────────────
volatile int rotPosition = 512;
volatile int lastCLK     = HIGH;
const int    ROT_MIN     = 0;
const int    ROT_MAX     = 1023;
const int    ROT_STEP    = 8;

int  currentMode  = 0;
int  lastZigStep  = 1;
int  lastSnapIdx  = 0;
bool musicPlaying = false;

unsigned long btnShapeTime=0, btnFeelTime=0, btnMusicTime=0, joySWTime=0;
bool btnShapePrev=HIGH, btnFeelPrev=HIGH, btnMusicPrev=HIGH, joySWPrev=HIGH;
const unsigned long DEBOUNCE_MS = 40;

const int SEND_INTERVAL  = 16;
const int SMOOTH_SAMPLES = 4;

WebSocketsServer ws(81);
int  connectedClient = -1;
long lastSend        = 0;

int slBuf[8]={0}, jxBuf[8]={0}, jyBuf[8]={0};
int slIdx=0, jxIdx=0, jyIdx=0;
bool centerCalibrated = false;

// ── Rotary interrupt ──────────────────────────────────────────────────────────
void IRAM_ATTR onRotaryChange() {
  int clk = digitalRead(ROT_CLK);
  int dt  = digitalRead(ROT_DT);
  if (clk != lastCLK) {
    rotPosition = (dt != clk)
      ? min(ROT_MAX, rotPosition + ROT_STEP)
      : max(ROT_MIN, rotPosition - ROT_STEP);
    lastCLK = clk;
  }
}

// ── Smooth ADC ────────────────────────────────────────────────────────────────
int smoothRead(int pin, int* buf, int &idx) {
  buf[idx % SMOOTH_SAMPLES] = analogRead(pin);
  idx++;
  long sum = 0;
  int cnt = min(idx, SMOOTH_SAMPLES);
  for (int i = 0; i < cnt; i++) sum += buf[i];
  return (int)(sum / cnt);
}

// ── Reed switches ─────────────────────────────────────────────────────────────
int readZigzag() {
  for (int i = ZIGZAG_COUNT-1; i >= 0; i--)
    if (digitalRead(ZIGZAG_PINS[i]) == LOW) return i + 1;
  return 0;
}

int readSnap() {
  for (int i = 0; i < SNAP_COUNT; i++)
    if (digitalRead(SNAP_PINS[i]) == LOW) return i;
  return -1;
}

void updateLEDs() {
  digitalWrite(LED_SHAPE, currentMode == 0 ? HIGH : LOW);
  digitalWrite(LED_FEEL,  currentMode == 1 ? HIGH : LOW);
}

// ── WebSocket events ──────────────────────────────────────────────────────────
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    connectedClient = num;
    char init[96];
    snprintf(init, sizeof(init),
      "{\"mode\":%d,\"zig\":%d,\"snap\":%d,\"music\":%d}",
      currentMode, lastZigStep, lastSnapIdx, musicPlaying ? 1 : 0);
    ws.sendTXT(connectedClient, init);
    updateLEDs();
    Serial.printf("[WS] Client #%u connected\n", num);
  } else if (type == WStype_DISCONNECTED) {
    if (connectedClient == num) connectedClient = -1;
    Serial.printf("[WS] Client #%u disconnected\n", num);
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\nLight Soul — ESP32-S3");

  analogSetAttenuation(ADC_11db);
  pinMode(SLIDER_PIN, INPUT);
  pinMode(JOY_X_PIN,  INPUT);
  pinMode(JOY_Y_PIN,  INPUT);
  pinMode(JOY_SW_PIN, INPUT_PULLUP);

  pinMode(ROT_CLK, INPUT_PULLUP);
  pinMode(ROT_DT,  INPUT_PULLUP);
  lastCLK = digitalRead(ROT_CLK);
  attachInterrupt(digitalPinToInterrupt(ROT_CLK), onRotaryChange, CHANGE);

  for (int i = 0; i < ZIGZAG_COUNT; i++) pinMode(ZIGZAG_PINS[i], INPUT_PULLUP);
  for (int i = 0; i < SNAP_COUNT;   i++) pinMode(SNAP_PINS[i],   INPUT_PULLUP);

  pinMode(BTN_SHAPE, INPUT_PULLUP);
  pinMode(BTN_FEEL,  INPUT_PULLUP);
  pinMode(BTN_MUSIC, INPUT_PULLUP);
  pinMode(LED_SHAPE, OUTPUT);
  pinMode(LED_FEEL,  OUTPUT);
  updateLEDs();

  // Auto-calibrate joystick center on boot (takes 20 samples, ~200ms)
  // Make sure joystick is untouched during boot!
  Serial.println("Calibrating joystick center — don't touch it...");
  long sumX=0, sumY=0;
  for (int i=0; i<20; i++) { sumX+=analogRead(JOY_X_PIN); sumY+=analogRead(JOY_Y_PIN); delay(10); }
  JX_CTR = (int)(sumX/20);
  JY_CTR = (int)(sumY/20);
  Serial.printf("Joystick center: JX=%d  JY=%d\n", JX_CTR, JY_CTR);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  int apWait = 0;
  while (WiFi.softAPIP().toString() == "0.0.0.0" && apWait < 20) { delay(200); apWait++; }

  Serial.println("\n=============================");
  Serial.println("  WiFi: " + String(AP_SSID) + "  pw: " + String(AP_PASSWORD));
  Serial.println("  URL:  http://" + WiFi.softAPIP().toString());
  Serial.println("  WS:   ws://"  + WiFi.softAPIP().toString() + ":81");
  Serial.println("=============================\n");

  ws.begin();
  ws.onEvent(onWsEvent);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  ws.loop();
  unsigned long now = millis();

  // Buttons
  bool shapeNow=digitalRead(BTN_SHAPE), feelNow=digitalRead(BTN_FEEL), musicNow=digitalRead(BTN_MUSIC);

  if (shapeNow==LOW && btnShapePrev==HIGH && (now-btnShapeTime)>DEBOUNCE_MS) {
    btnShapeTime=now; currentMode=0; updateLEDs();
    if (connectedClient>=0) ws.sendTXT(connectedClient, "{\"btn\":\"shape\"}");
  }
  btnShapePrev=shapeNow;

  if (feelNow==LOW && btnFeelPrev==HIGH && (now-btnFeelTime)>DEBOUNCE_MS) {
    btnFeelTime=now; currentMode=1; updateLEDs();
    if (connectedClient>=0) ws.sendTXT(connectedClient, "{\"btn\":\"feel\"}");
  }
  btnFeelPrev=feelNow;

  if (musicNow==LOW && btnMusicPrev==HIGH && (now-btnMusicTime)>DEBOUNCE_MS) {
    btnMusicTime=now; musicPlaying=!musicPlaying;
    ws.sendTXT(connectedClient, musicPlaying
      ? "{\"btn\":\"music\",\"action\":\"play\"}"
      : "{\"btn\":\"music\",\"action\":\"stop\"}");
  }
  btnMusicPrev=musicNow;

  // Joystick SW re-center
  bool joySWNow=digitalRead(JOY_SW_PIN);
  if (joySWNow==LOW && joySWPrev==HIGH && (now-joySWTime)>DEBOUNCE_MS) {
    joySWTime=now;
    if (connectedClient>=0) ws.sendTXT(connectedClient, "{\"btn\":\"joycenter\"}");
  }
  joySWPrev=joySWNow;

  // Reed switches
  int zigNow=readZigzag(), snapNow=readSnap();
  if (zigNow  > 0)  lastZigStep=zigNow;
  if (snapNow >= 0) lastSnapIdx=snapNow;

  // Sensor packet
  if ((long)(now-lastSend) >= SEND_INTERVAL) {
    lastSend=now;

    int sl  = smoothRead(SLIDER_PIN, slBuf, slIdx);
    int jx  = smoothRead(JOY_X_PIN,  jxBuf, jxIdx);
    int jy  = smoothRead(JOY_Y_PIN,  jyBuf, jyIdx);
    int rot = rotPosition;

    Serial.printf("SL:%4d  JX:%4d(c:%4d)  JY:%4d(c:%4d)  ROT:%4d  ZIG:%d  SNAP:%d\n",
      sl, jx, JX_CTR, jy, JY_CTR, rot, lastZigStep, lastSnapIdx);

    if (connectedClient >= 0) {
      char msg[160];
      // Always send all values — HTML holds last value naturally, no reset to center
      snprintf(msg, sizeof(msg),
        "{\"sl\":%d,\"jx\":%d,\"jy\":%d,\"rot\":%d,\"zig\":%d,\"snap\":%d,\"jxc\":%d,\"jyc\":%d}",
        sl, jx, jy, rot, lastZigStep, lastSnapIdx, JX_CTR, JY_CTR);
      ws.sendTXT(connectedClient, msg);
    }
  }
}
