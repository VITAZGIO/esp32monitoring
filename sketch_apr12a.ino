#include <Arduino.h>
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <RCSwitch.h>

// === DS18B20 (датчик БП) ===
#include <OneWire.h>
#include <DallasTemperature.h>

// =====================================================
// =============== НАСТРОЙКИ / ПИНЫ =====================
// =====================================================

// Wi-Fi
static const char* WIFI_SSID = "PONT228";
static const char* WIFI_PASS = "2284201337";

// MQTT
static const char* MQTT_HOST = "192.168.1.195";
static const uint16_t MQTT_PORT = 1883;

// RF433 (как у тебя)
static const int RF_RX_PIN = 27;
static const int RF_TX_PIN = 26;

// Реле (как у тебя)
static const int RELAY1_PIN = 13;
static const int RELAY2_PIN = 14;
static const int RELAY3_PIN = 25;
static const int RELAY4_PIN = 33;

// Спикер (как у тебя)
static const int BUZZER_PIN = 32;

// Кнопки (старые ADC) (как у тебя)
static const int PIN_A = 34; // кнопки 1..3
static const int PIN_B = 35; // кнопки 4..6

// Кнопки (новые ADC лестница на GPIO39)
static const int PIN_C = 39; // кнопки 7..12

// DS18B20 (датчик БП)
static const int PSU_TEMP_PIN = 21;

// -----------------------------------------------------
// MQTT топики (монитор/яркость/реле/спикер) — НЕ МЕНЯЮ
// -----------------------------------------------------

// Монитор данные
static const char* T_OP_TEMP = "esp32panel/orange_pi/temp";
static const char* T_OP_CPU  = "esp32panel/orange_pi/cpu";
static const char* T_OP_RAM  = "esp32panel/orange_pi/ram";

static const char* T_MP_TEMP = "esp32panel/proxmox/temp";
static const char* T_MP_CPU  = "esp32panel/proxmox/cpu";
static const char* T_MP_RAM  = "esp32panel/proxmox/ram";

// Яркость (не меняю)
static const char* T_BRIGHT_SET   = "esp32panel/screen/brightness/set";
static const char* T_BRIGHT_STATE = "esp32panel/screen/brightness/state";

// Реле (не меняю)
static const char* T_RELAY1_SET   = "esp32panel/relay/1/set";
static const char* T_RELAY2_SET   = "esp32panel/relay/2/set";
static const char* T_RELAY3_SET   = "esp32panel/relay/3/set";
static const char* T_RELAY4_SET   = "esp32panel/relay/4/set";

static const char* T_RELAY1_STATE = "esp32panel/relay/1/state";
static const char* T_RELAY2_STATE = "esp32panel/relay/2/state";
static const char* T_RELAY3_STATE = "esp32panel/relay/3/state";
static const char* T_RELAY4_STATE = "esp32panel/relay/4/state";

// Спикер топик (твой!)
static const char* T_SPK_PLAY = "esp32panel/speaker/play";

// Таймауты (онлайн по данным)
static const uint32_t DATA_TIMEOUT_MS = 60000;

// -----------------------------------------------------
// MQTT топики RF433 — КАК В ТВОЁМ radio.txt (НЕ МЕНЯЮ)
// -----------------------------------------------------
static const char* TOPIC_TX       = "rf433/tx";
static const char* TOPIC_STATUS   = "rf433/status";
static const char* TOPIC_LAST     = "rf433/last_code";
static const char* TOPIC_LAST_RAW = "rf433/last_code_raw";

// отдельная команда hold97 (не меняю)
static const char* TOPIC_HOLD97   = "rf433/tx_hold97";

// RF protocol settings (твои)
static const int RF_PROTOCOL = 1;
static const int RF_PULSE    = 389;
static const int RF_BITS     = 24;

// 8 кодов-триггеров (твои)
static const uint32_t TRIGGERS[] = {
  1382424, 1382420, 1382428, 1382418,
  1382426, 1382422, 1382423, 1382431
};
static const int TRIGGERS_N = sizeof(TRIGGERS) / sizeof(TRIGGERS[0]);

// 21 код подсветки (твои)
static const uint32_t LIGHTS[] = {
  9348097, 9348098, 9348099, 9348100, 9348101, 9348102, 9348103,
  9348104, 9348105, 9348106, 9348107, 9348108, 9348109, 9348110,
  9348111, 9348112, 9348113, 9348114, 9348115, 9348116, 9348117
};
static const int LIGHTS_N = sizeof(LIGHTS) / sizeof(LIGHTS[0]);

// =====================================================
// ================== FAN PWM ===========================
// =====================================================

static const int FAN_PWM_PIN = 17;

static const int PWM_FREQ = 25000;
static const int PWM_RESOLUTION = 8; // 0..255

static const char* T_FAN_SET       = "esp32panel/fan/set";
static const char* T_FAN_STATE     = "esp32panel/fan/state";
static const char* T_FAN_PWM_STATE = "esp32panel/fan/pwm_state";

// 10 скоростей:
// 1 = маленькая
// 10 = большая
// Инверсная логика: чем меньше значение, тем выше скорость
static const uint8_t FAN_SPEED_PWM[10] = {
  230, // 1
  205, // 2
  180, // 3
  155, // 4
  130, // 5
  105, // 6
   80, // 7
   55, // 8
   30, // 9
    0  // 10
};

static int currentSpeedLevel = 4; // стартовая скорость

// =====================================================
// =============== ТИПЫ =================================
// =====================================================

struct Note { uint16_t f; uint16_t d; uint16_t gap; };

enum BrightMode : uint8_t { BR_OFF=0, BR_DIM=1, BR_ON=2 };

struct ServerData {
  const char* name;
  bool online;
  int tempC;
  int cpu;
  int ram;
};

struct State {
  bool wifiOk;
  bool haOk;
  ServerData a;
  ServerData b;
};

// =====================================================
// ================== ГЛОБАЛЫ ===========================
// =====================================================

WiFiClient espClient;
PubSubClient mqtt(espClient);

TFT_eSPI tft;
static const uint8_t  ROT = 1;
static const uint16_t BG  = TFT_BLACK;

static const int VALUE_X_OFF   = 42;
static const int VALUE_H       = 10;
static const int VALUE_R_PAD   = 6;

static uint32_t lastAnyDataMs = 0;
static uint32_t lastOPDataMs  = 0;
static uint32_t lastMPDataMs  = 0;

State cur = { false, false, {"ORANGE PI", false, 0, 0, 0}, {"PROXMOX", false, 0, 0, 0} };
State prev = cur;

static bool dirty = true;
static uint32_t lastUiTick = 0;

// Brightness pin from TFT_eSPI
#ifdef TFT_BL
static const int BL_PIN = TFT_BL;
#else
static const int BL_PIN = -1;
#endif

static BrightMode brightMode = BR_ON;

// Relay states
static bool relayState[4] = {false,false,false,false};
static bool     relayPulseActive[4] = {false,false,false,false};
static uint32_t relayPulseOffAt[4]  = {0,0,0,0};

// Speaker player (non-blocking)
static const Note* spkSeq = nullptr;
static uint16_t spkIdx = 0;
static bool spkToneOn = false;
static uint32_t spkNextAt = 0;

// RF433
RCSwitch rf = RCSwitch();
static uint32_t rfLastStatusMs = 0;

// антидребезг trigger (оставляю как в твоём коде: 1000)
static uint32_t lastTriggerMs = 0;
static const uint32_t TRIGGER_BLOCK_MS = 1000;

// hold97 (оставляю как было)
static const uint32_t HOLD97_CODE = 9348097;
static const uint32_t HOLD97_DURATION_MS = 2500;
static const uint32_t HOLD97_INTERVAL_MS = 250;
static bool hold97Active = false;
static uint32_t hold97UntilMs = 0;
static uint32_t hold97NextSendMs = 0;

// =====================================================
// ================== КНОПКИ (12 шт + DOUBLE) ===========
// =====================================================

// Старые 6 кнопок: thresholds
static const int TH_220  = 220;
static const int TH_1K   = 900;
static const int TH_10K  = 3000;

// Debounce
static int stableBtn = 0;      // 0 = none, 1..12
static int lastReadBtn = 0;
static uint32_t lastChangeMs = 0;
static const uint32_t DEBOUNCE_MS = 50;

// анти-спам на PRESS
static uint32_t lastPressMs[13] = {0};        // 1..12
static const uint32_t PRESS_COOLDOWN_MS = 250;

// double press (2 клика за 2 секунды)
static uint32_t lastPressForDoubleMs[13] = {0};
static const uint32_t DOUBLE_WINDOW_MS = 2000;

// Новая лестница на GPIO39: твои измеренные резисторы (Ом)
static const float LADDER_PULLUP = 10000.0f; // 10k к 3.3V
static const float LADDER_R[6] = { 677.0f, 2000.0f, 3266.0f, 6782.0f, 19920.0f, 67430.0f };

static const int LADDER_SAMPLES = 32;
static const int LADDER_MARGIN = 260;     // допуск по RAW (если что — подправим)
static const int LADDER_NOISE_FLOOR = 80;

// MQTT publish (СУЩЕСТВУЮЩИЕ НЕ ЛОМАЮ)
static void mqttPublishPress(int btn) {
  if (!mqtt.connected()) return;
  char topic[64];
  snprintf(topic, sizeof(topic), "esp32panel/buttons/btn%d", btn); // btn1..btn12
  mqtt.publish(topic, "PRESS", false);
}

static void mqttPublishDouble(int btn) {
  if (!mqtt.connected()) return;
  char topic[80];
  snprintf(topic, sizeof(topic), "esp32panel/buttons/btn%d/double", btn); // НОВОЕ
  mqtt.publish(topic, "DOUBLE", false);
}

// старые ADC чтения
static int readAdcAvgSimple(int pin) {
  long s = 0;
  for (int i = 0; i < 8; i++) { s += analogRead(pin); delay(1); }
  return (int)(s / 8);
}

static int decode3Keys(int adc) {
  if (adc < TH_220) return 1;
  if (adc < TH_1K)  return 2;
  if (adc < TH_10K) return 3;
  return 0;
}

// лестница GPIO39
static int readAdcAvgLadder(int pin) {
  uint32_t sum = 0;
  for (int i = 0; i < LADDER_SAMPLES; i++) {
    sum += analogRead(pin);
    delayMicroseconds(300);
  }
  return (int)(sum / LADDER_SAMPLES);
}

static int expectedRaw(float vref, float rPullup, float rbtn) {
  float v = vref * (rbtn / (rPullup + rbtn));
  return (int)lround((v / vref) * 4095.0f);
}

static int decodeLadder6(int raw) {
  if (raw < LADDER_NOISE_FLOOR) return 0;
  const float VREF = 3.3f;

  int bestIdx = -1;
  int bestDiff = 999999;
  for (int i = 0; i < 6; i++) {
    int er = expectedRaw(VREF, LADDER_PULLUP, LADDER_R[i]);
    int diff = abs(raw - er);
    if (diff < bestDiff) { bestDiff = diff; bestIdx = i; }
  }
  if (bestIdx >= 0 && bestDiff <= LADDER_MARGIN) return bestIdx + 1; // 1..6
  return 0;
}

// 0..12
static int decodeButton12() {
  // старые 1..6
  int a = readAdcAvgSimple(PIN_A);
  int b = readAdcAvgSimple(PIN_B);

  int ka = decode3Keys(a);
  if (ka) return ka;         // 1..3

  int kb = decode3Keys(b);
  if (kb) return kb + 3;     // 4..6

  // новые 7..12
  int c = readAdcAvgLadder(PIN_C);
  int kc = decodeLadder6(c);
  if (kc) return kc + 6;     // 7..12

  return 0;
}

static void handlePressEvent(int btn) {
  mqttPublishPress(btn); // всегда (совместимость)

  uint32_t now = millis();
  uint32_t last = lastPressForDoubleMs[btn];
  if (last != 0 && (now - last) <= DOUBLE_WINDOW_MS) {
    mqttPublishDouble(btn);
    lastPressForDoubleMs[btn] = 0; // сброс
  } else {
    lastPressForDoubleMs[btn] = now;
  }
}

static void buttonsTick() {
  int k = decodeButton12();

  if (k != lastReadBtn) { lastReadBtn = k; lastChangeMs = millis(); }

  if (millis() - lastChangeMs > DEBOUNCE_MS) {
    if (stableBtn != k) {
      stableBtn = k;

      // только нажатие (0 -> btn)
      if (stableBtn != 0) {
        int btn = stableBtn;
        uint32_t now = millis();
        if (now - lastPressMs[btn] >= PRESS_COOLDOWN_MS) {
          lastPressMs[btn] = now;
          handlePressEvent(btn);
        }
      }
    }
  }
}

// =====================================================
// ================== СПИКЕР СЕКВЕНЦИИ =================
// =====================================================

static const Note SEQ_BEEP_2500[] = { {2500,120,120}, {0,0,0} };
static const Note SEQ_SOFT_800[]  = { {800,200,120},  {0,0,0} };
static const Note SEQ_ERROR_600[] = { {600,400,120},  {0,0,0} };
static const Note SEQ_NOTIFY[]    = { {1800,150,120}, {2200,150,120}, {0,0,0} };
static const Note SEQ_BOOT[]      = { {1200,100,120}, {1800,100,120}, {2400,100,120}, {0,0,0} };
static const Note SEQ_WARN[]      = { {1000,150,120}, {1000,150,120}, {1000,150,120}, {0,0,0} };
static const Note SEQ_SUCCESS[]   = { {2000, 80,120}, {3000,120,120}, {0,0,0} };

static const Note SEQ_MARIO[] = {
  {2637,150,120},{2637,150,90},{2637,150,120},
  {2093,150,120},{2637,200,170},{3136,320,150},{1568,320,220},
  {2093,200,120},{1568,200,120},{1319,260,120},
  {1760,200,120},{1976,200,120},{1865,200,120},{1760,260,90},
  {1568,160,120},{1760,160,120},{1976,160,120},{2093,200,120},
  {0,0,0}
};

static const Note SEQ_STARWARS[] = {
  {1960,380,120},{1960,380,120},{1960,380,120},
  {1568,260,120},{2349,160,120},{1960,420,120},
  {1568,260,120},{2349,160,120},{1960,520,120},
  {2937,360,120},{2937,360,120},{2937,360,120},
  {3136,260,120},{2349,160,120},{1865,420,120},
  {1568,260,120},{2349,160,120},{1960,650,120},
  {0,0,0}
};

// =====================================================
// ================== HELPERS ===========================
// =====================================================

static int parseIntSafe(const char* s, int def = 0) {
  if (!s || !*s) return def;
  return (int)strtof(s, nullptr);
}

static BrightMode parseBrightMode(const char* s) {
  if (!s) return BR_ON;
  if (!strcasecmp(s, "OFF")) return BR_OFF;
  if (!strcasecmp(s, "DIM")) return BR_DIM;
  return BR_ON;
}

static inline void markOPDataSeen() { lastOPDataMs = millis(); lastAnyDataMs = lastOPDataMs; }
static inline void markMPDataSeen() { lastMPDataMs = millis(); lastAnyDataMs = lastMPDataMs; }

static void resetServerData(ServerData& s) { s.tempC = 0; s.cpu = 0; s.ram = 0; }

static int parsePulseMs(const char* s, int defMs = 300) {
  if (!s) return -1;
  if (strncasecmp(s, "PULSE", 5) != 0) return -1;
  if (s[5] == 0) return defMs;
  if (s[5] == '_') {
    int v = atoi(s + 6);
    if (v <= 0) v = defMs;
    v = constrain(v, 50, 5000);
    return v;
  }
  return defMs;
}

static bool inList(uint32_t v, const uint32_t* arr, int n) {
  for (int i = 0; i < n; i++) if (arr[i] == v) return true;
  return false;
}

static void rfPublishOnline() {
  if (!mqtt.connected()) return;
  mqtt.publish(TOPIC_STATUS, "{\"online\":true}", true);
}

// =====================================================
// ================== FAN HELPERS =======================
// =====================================================

static void publishFanState() {
  if (!mqtt.connected()) return;

  char buf[16];

  snprintf(buf, sizeof(buf), "%d", currentSpeedLevel);
  mqtt.publish(T_FAN_STATE, buf, true);

  snprintf(buf, sizeof(buf), "%d", FAN_SPEED_PWM[currentSpeedLevel - 1]);
  mqtt.publish(T_FAN_PWM_STATE, buf, true);
}

static void applySpeedLevel(int level) {
  level = constrain(level, 1, 10);
  currentSpeedLevel = level;

  uint8_t pwm = FAN_SPEED_PWM[level - 1];
  ledcWrite(FAN_PWM_PIN, pwm);

  Serial.print("Fan speed level = ");
  Serial.print(level);
  Serial.print(" | PWM = ");
  Serial.println(pwm);

  publishFanState();
}

// =====================================================
// ================== BRIGHTNESS ========================
// =====================================================

static void applyBrightness(BrightMode m) {
  brightMode = m;
  if (BL_PIN < 0) return;
  pinMode(BL_PIN, OUTPUT);

  if (m == BR_OFF) digitalWrite(BL_PIN, !TFT_BACKLIGHT_ON);
  else             digitalWrite(BL_PIN, TFT_BACKLIGHT_ON);
}

static void publishBrightnessState() {
  if (!mqtt.connected()) return;
  const char* s = "ON";
  if (brightMode == BR_OFF) s = "OFF";
  if (brightMode == BR_DIM) s = "DIM";
  mqtt.publish(T_BRIGHT_STATE, s, true);
}

// =====================================================
// ================== RELAYS ============================
// =====================================================

static void relayWriteHW(int idx, bool on) {
  int pin = (idx==0)?RELAY1_PIN:(idx==1)?RELAY2_PIN:(idx==2)?RELAY3_PIN:RELAY4_PIN;
  digitalWrite(pin, on ? LOW : HIGH);
}

static void publishRelayState(int idx) {
  if (!mqtt.connected()) return;
  const char* topic = (idx==0)?T_RELAY1_STATE:(idx==1)?T_RELAY2_STATE:(idx==2)?T_RELAY3_STATE:T_RELAY4_STATE;
  mqtt.publish(topic, relayState[idx] ? "ON" : "OFF", true);
}

static void relaySet(int idx, bool on) {
  relayState[idx] = on;
  relayWriteHW(idx, on);
  publishRelayState(idx);
}

static void relayToggle(int idx) { relaySet(idx, !relayState[idx]); }

static void relayPulse(int idx, int ms) {
  relaySet(idx, true);
  relayPulseActive[idx] = true;
  relayPulseOffAt[idx]  = millis() + (uint32_t)ms;
}

// =====================================================
// ================== SPEAKER ===========================
// =====================================================

static void spkStop() {
  noTone(BUZZER_PIN);
  spkSeq = nullptr;
  spkIdx = 0;
  spkToneOn = false;
  spkNextAt = 0;
}

static void spkStart(const Note* seq) { spkStop(); spkSeq = seq; }

static void spkTick() {
  if (!spkSeq) return;
  uint32_t now = millis();
  if (spkNextAt && (int32_t)(now - spkNextAt) < 0) return;

  Note n = spkSeq[spkIdx];
  if (n.f == 0 && n.d == 0 && n.gap == 0) { spkStop(); return; }

  if (!spkToneOn) {
    tone(BUZZER_PIN, n.f);
    spkToneOn = true;
    spkNextAt = now + n.d;
  } else {
    noTone(BUZZER_PIN);
    spkToneOn = false;
    spkNextAt = now + n.gap;
    spkIdx++;
  }
}

// =====================================================
// ================== UI COLORS =========================
// =====================================================

static uint16_t colStatus(bool ok) { return ok ? TFT_GREEN : TFT_RED; }

static uint16_t colByPctGood(int pct) {
  pct = constrain(pct, 0, 100);
  if (pct >= 85) return TFT_RED;
  if (pct >= 70) return TFT_ORANGE;
  if (pct >= 55) return TFT_YELLOW;
  if (pct >= 40) return TFT_GREENYELLOW;
  return TFT_GREEN;
}

static uint16_t colByTemp(int t) {
  if (t >= 75) return TFT_RED;
  if (t >= 65) return TFT_ORANGE;
  if (t >= 55) return TFT_YELLOW;
  if (t >= 45) return TFT_GREENYELLOW;
  return TFT_GREEN;
}

// =====================================================
// ================== UI DRAW ===========================
// =====================================================

int W, H;
int WIFI_X, WIFI_Y;
int HA_X, HA_Y, HA_W, HA_H;
int topY;
int colX[2], colY[2], colW, colH;
int pad = 3;
int gap = 4;

int nameLift = 3;
int textLift = 2;

int nameYoff;
int tempLabelYoff, tempBarYoff;
int cpuLabelYoff,  cpuBarYoff;
int ramLabelYoff,  ramBarYoff;

int labelH = 10;
int barH   = 8;
int between = 3;
int labelToBar = 2;
int onlineH = 12;

static void drawWifiIcon(int x, int y, bool ok) {
  uint16_t c = ok ? TFT_WHITE : TFT_RED;
  tft.fillRect(x, y, 22, 18, BG);

  int cx = x + 10;
  int cy = y + 10;

  tft.fillCircle(cx, cy + 6, 1, c);

  tft.drawLine(cx - 3, cy + 4, cx + 3, cy + 4, c);
  tft.drawLine(cx - 4, cy + 3, cx - 3, cy + 4, c);
  tft.drawLine(cx + 4, cy + 3, cx + 3, cy + 4, c);

  tft.drawLine(cx - 6, cy + 1, cx + 6, cy + 1, c);
  tft.drawLine(cx - 7, cy + 0, cx - 6, cy + 1, c);
  tft.drawLine(cx + 7, cy + 0, cx + 6, cy + 1, c);

  tft.drawLine(cx - 9, cy - 2, cx + 9, cy - 2, c);
  tft.drawLine(cx - 10, cy - 3, cx - 9, cy - 2, c);
  tft.drawLine(cx + 10, cy - 3, cx + 9, cy - 2, c);
}

static void drawStatusBar(int x, int y, int w, int h, const char* label, bool ok) {
  uint16_t fill = colStatus(ok);
  tft.fillRoundRect(x, y, w, h, 4, fill);
  tft.drawRoundRect(x, y, w, h, 4, TFT_WHITE);

  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_BLACK, fill);
  tft.drawString(label, x + w/2, y + h/2);
}

static void drawBar(int x, int y, int w, int h, int pct, uint16_t fillColor) {
  pct = constrain(pct, 0, 100);
  tft.drawRect(x, y, w, h, TFT_WHITE);

  int innerW = w - 2;
  int fillW  = (innerW * pct) / 100;

  tft.fillRect(x + 1, y + 1, fillW, h - 2, fillColor);
  if (fillW < innerW) tft.fillRect(x + 1 + fillW, y + 1, innerW - fillW, h - 2, BG);
}

static void drawTextLine(int x, int y, int w, int h, const char* s) {
  tft.fillRect(x, y, w, h, BG);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, BG);
  tft.drawString(s, x, y);
}

static void buildLayout() {
  W = tft.width();
  H = tft.height();

  WIFI_X = W - 22;
  WIFI_Y = 2;

  HA_X = 2;
  HA_Y = 2;
  HA_H = 18;
  HA_W = W - 2 - 2 - 22 - 4;

  topY = HA_Y + HA_H + 4;

  colW = (W - 2*2 - gap) / 2;
  colH = H - topY - 2;

  colX[0] = 2;
  colY[0] = topY;

  colX[1] = 2 + colW + gap;
  colY[1] = topY;

  int curY = pad;

  nameYoff = curY - nameLift;
  curY += 14;

  tempLabelYoff = curY - textLift;
  curY += labelH + labelToBar;

  tempBarYoff = curY;
  curY += barH + between;

  cpuLabelYoff = curY - textLift;
  curY += labelH + labelToBar;

  cpuBarYoff = curY;
  curY += barH + between;

  ramLabelYoff = curY - textLift;
  curY += labelH + labelToBar;

  ramBarYoff = curY;
}

static void drawColumnStatic(int idx, const ServerData& s) {
  int x = colX[idx], y = colY[idx];

  tft.drawRect(x, y, colW, colH, TFT_DARKGREY);

  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_WHITE, BG);
  tft.drawString(s.name, x + pad, y + nameYoff);

  tft.drawString("TEMP", x + pad, y + tempLabelYoff);
  tft.drawString("CPU",  x + pad, y + cpuLabelYoff);
  tft.drawString("RAM",  x + pad, y + ramLabelYoff);

  int barX = x + pad;
  int barW = colW - pad*2;
  tft.drawRect(barX, y + tempBarYoff, barW, barH, TFT_WHITE);
  tft.drawRect(barX, y + cpuBarYoff,  barW, barH, TFT_WHITE);
  tft.drawRect(barX, y + ramBarYoff,  barW, barH, TFT_WHITE);

  int onlineX = x + pad;
  int onlineW = colW - pad*2;
  int onlineY = y + colH - onlineH - pad;

  drawStatusBar(onlineX, onlineY, onlineW, onlineH, s.online ? "ONLINE" : "OFFLINE", s.online);
}

static void drawTopStatic() {
  tft.fillScreen(BG);
  drawWifiIcon(WIFI_X, WIFI_Y, cur.wifiOk);
  drawStatusBar(HA_X, HA_Y, HA_W, HA_H, "Home Assistant", cur.haOk);
  drawColumnStatic(0, cur.a);
  drawColumnStatic(1, cur.b);
}

static void updateColumnDynamic(int idx, const ServerData& s, const ServerData& p, bool forceAll) {
  int x = colX[idx], y = colY[idx];
  int barX = x + pad;
  int barW = colW - pad*2;

  int valueX = x + pad + VALUE_X_OFF;
  int valueW = colW - (pad + VALUE_X_OFF) - VALUE_R_PAD;

  if (forceAll || s.tempC != p.tempC) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%dC", s.tempC);
    drawTextLine(valueX, y + tempLabelYoff, valueW, VALUE_H, buf);

    int tp = map(constrain(s.tempC, 20, 90), 20, 90, 0, 100);
    drawBar(barX, y + tempBarYoff, barW, barH, tp, colByTemp(s.tempC));
  }

  if (forceAll || s.cpu != p.cpu) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%d%%", s.cpu);
    drawTextLine(valueX, y + cpuLabelYoff, valueW, VALUE_H, buf);
    drawBar(barX, y + cpuBarYoff, barW, barH, s.cpu, colByPctGood(s.cpu));
  }

  if (forceAll || s.ram != p.ram) {
    char buf[12];
    snprintf(buf, sizeof(buf), "%d%%", s.ram);
    drawTextLine(valueX, y + ramLabelYoff, valueW, VALUE_H, buf);
    drawBar(barX, y + ramBarYoff, barW, barH, s.ram, colByPctGood(s.ram));
  }

  if (forceAll || s.online != p.online) {
    int onlineX = x + pad;
    int onlineW = colW - pad*2;
    int onlineY = y + colH - onlineH - pad;
    drawStatusBar(onlineX, onlineY, onlineW, onlineH, s.online ? "ONLINE" : "OFFLINE", s.online);
  }
}

static void updateTopDynamic(bool forceAll) {
  if (forceAll || cur.wifiOk != prev.wifiOk) drawWifiIcon(WIFI_X, WIFI_Y, cur.wifiOk);
  if (forceAll || cur.haOk   != prev.haOk)   drawStatusBar(HA_X, HA_Y, HA_W, HA_H, "Home Assistant", cur.haOk);
  updateColumnDynamic(0, cur.a, prev.a, forceAll);
  updateColumnDynamic(1, cur.b, prev.b, forceAll);
}

// =====================================================
// ================== DS18B20 (датчик БП) ==============
// =====================================================

static const char* T_PSU_TEMP_STATE = "esp32panel/psu/temp"; // НОВОЕ

OneWire oneWire(PSU_TEMP_PIN);
DallasTemperature psuSensors(&oneWire);

static uint32_t psuNextReadMs = 0;
static bool psuDiscoverySent = false;

static void psuPublishDiscovery() {
  if (!mqtt.connected() || psuDiscoverySent) return;

  String uid = "esp32panel_psu_temp_";
  uid += String((uint32_t)ESP.getEfuseMac(), HEX);

  // фиксированный discovery topic (retained)
  const char* topic = "homeassistant/sensor/esp32panel/psu_temp/config";

  String payload = "{";
  payload += "\"name\":\"датчик бп\",";
  payload += "\"unique_id\":\"" + uid + "\",";
  payload += "\"state_topic\":\"" + String(T_PSU_TEMP_STATE) + "\",";
  payload += "\"unit_of_measurement\":\"°C\",";
  payload += "\"device_class\":\"temperature\",";
  payload += "\"state_class\":\"measurement\",";
  payload += "\"expire_after\":120";
  payload += "}";

  mqtt.publish(topic, payload.c_str(), true);
  psuDiscoverySent = true;
}

static void psuTick() {
  uint32_t now = millis();
  if ((int32_t)(now - psuNextReadMs) < 0) return;
  psuNextReadMs = now + 2000;

  psuSensors.requestTemperatures();
  float t = psuSensors.getTempCByIndex(0);
  if (t == DEVICE_DISCONNECTED_C) return;

  if (mqtt.connected()) {
    char buf[16];
    dtostrf(t, 0, 2, buf);
    mqtt.publish(T_PSU_TEMP_STATE, buf, true); // retained
  }
}

// =====================================================
// ================== MQTT CALLBACK =====================
// =====================================================

static void mqttCallback(char* topic, byte* payload, unsigned int len) {
  static char msg[256];
  unsigned int n = (len >= sizeof(msg)) ? sizeof(msg) - 1 : len;
  memcpy(msg, payload, n);
  msg[n] = 0;

  bool changed = false;

  // -------- monitor values --------
  if (!strcmp(topic, T_OP_TEMP)) {
    int v = parseIntSafe(msg, cur.a.tempC);
    if (v != cur.a.tempC) { cur.a.tempC = v; changed = true; }
    markOPDataSeen();
  }
  else if (!strcmp(topic, T_OP_CPU)) {
    int v = constrain(parseIntSafe(msg, cur.a.cpu), 0, 100);
    if (v != cur.a.cpu) { cur.a.cpu = v; changed = true; }
    markOPDataSeen();
  }
  else if (!strcmp(topic, T_OP_RAM)) {
    int v = constrain(parseIntSafe(msg, cur.a.ram), 0, 100);
    if (v != cur.a.ram) { cur.a.ram = v; changed = true; }
    markOPDataSeen();
  }

  else if (!strcmp(topic, T_MP_TEMP)) {
    int v = parseIntSafe(msg, cur.b.tempC);
    if (v != cur.b.tempC) { cur.b.tempC = v; changed = true; }
    markMPDataSeen();
  }
  else if (!strcmp(topic, T_MP_CPU)) {
    int v = constrain(parseIntSafe(msg, cur.b.cpu), 0, 100);
    if (v != cur.b.cpu) { cur.b.cpu = v; changed = true; }
    markMPDataSeen();
  }
  else if (!strcmp(topic, T_MP_RAM)) {
    int v = constrain(parseIntSafe(msg, cur.b.ram), 0, 100);
    if (v != cur.b.ram) { cur.b.ram = v; changed = true; }
    markMPDataSeen();
  }

  // -------- brightness --------
  else if (!strcmp(topic, T_BRIGHT_SET)) {
    BrightMode m = parseBrightMode(msg);
    if (m != brightMode) {
      applyBrightness(m);
      publishBrightnessState();
      changed = true;
    }
  }

  // -------- fan pwm --------
  else if (!strcmp(topic, T_FAN_SET)) {
    int level = parseIntSafe(msg, currentSpeedLevel);
    applySpeedLevel(level);
  }

  // -------- relays + PULSE --------
  else if (!strcmp(topic, T_RELAY1_SET)) {
    int pms = parsePulseMs(msg, 300);
    if (pms > 0) relayPulse(0, pms);
    else if (!strcasecmp(msg, "TOGGLE")) relayToggle(0);
    else if (!strcasecmp(msg, "ON")) relaySet(0, true);
    else if (!strcasecmp(msg, "OFF")) relaySet(0, false);
  }
  else if (!strcmp(topic, T_RELAY2_SET)) {
    int pms = parsePulseMs(msg, 300);
    if (pms > 0) relayPulse(1, pms);
    else if (!strcasecmp(msg, "TOGGLE")) relayToggle(1);
    else if (!strcasecmp(msg, "ON")) relaySet(1, true);
    else if (!strcasecmp(msg, "OFF")) relaySet(1, false);
  }
  else if (!strcmp(topic, T_RELAY3_SET)) {
    int pms = parsePulseMs(msg, 300);
    if (pms > 0) relayPulse(2, pms);
    else if (!strcasecmp(msg, "TOGGLE")) relayToggle(2);
    else if (!strcasecmp(msg, "ON")) relaySet(2, true);
    else if (!strcasecmp(msg, "OFF")) relaySet(2, false);
  }
  else if (!strcmp(topic, T_RELAY4_SET)) {
    int pms = parsePulseMs(msg, 300);
    if (pms > 0) relayPulse(3, pms);
    else if (!strcasecmp(msg, "TOGGLE")) relayToggle(3);
    else if (!strcasecmp(msg, "ON")) relaySet(3, true);
    else if (!strcasecmp(msg, "OFF")) relaySet(3, false);
  }

  // -------- speaker --------
  else if (!strcmp(topic, T_SPK_PLAY)) {
    if (!strcasecmp(msg, "STOP"))           spkStop();
    else if (!strcasecmp(msg, "MARIO"))     spkStart(SEQ_MARIO);
    else if (!strcasecmp(msg, "STARWARS"))  spkStart(SEQ_STARWARS);
    else if (!strcasecmp(msg, "BEEP_2500")) spkStart(SEQ_BEEP_2500);
    else if (!strcasecmp(msg, "SOFT_800"))  spkStart(SEQ_SOFT_800);
    else if (!strcasecmp(msg, "ERROR_600")) spkStart(SEQ_ERROR_600);
    else if (!strcasecmp(msg, "NOTIFY"))    spkStart(SEQ_NOTIFY);
    else if (!strcasecmp(msg, "BOOT"))      spkStart(SEQ_BOOT);
    else if (!strcasecmp(msg, "WARN"))      spkStart(SEQ_WARN);
    else if (!strcasecmp(msg, "SUCCESS"))   spkStart(SEQ_SUCCESS);
  }

  // -------- RF433 TX --------
  else if (!strcmp(topic, TOPIC_TX)) {
    String s = String(msg);
    s.trim();
    if (s.length() > 0) {
      uint32_t code = (uint32_t)s.toInt();
      if (code != 0) {
        rf.send(code, RF_BITS);

        if (mqtt.connected()) {
          mqtt.publish(TOPIC_LAST, String(code).c_str(), false);

          String raw = "{\"value\":";
          raw += code;
          raw += ",\"bits\":";
          raw += RF_BITS;
          raw += ",\"proto\":";
          raw += RF_PROTOCOL;
          raw += ",\"pulse\":";
          raw += RF_PULSE;
          raw += ",\"is_trigger\":false}";
          mqtt.publish(TOPIC_LAST_RAW, raw.c_str(), false);
        }
      }
    }
  }
  // RF433 HOLD97
  else if (!strcmp(topic, TOPIC_HOLD97)) {
    uint32_t now = millis();
    hold97Active = true;
    hold97UntilMs = now + HOLD97_DURATION_MS;
    hold97NextSendMs = 0;
  }

  if (changed) dirty = true;
}

// =====================================================
// ================== CONNECT ===========================
// =====================================================

static void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

static bool connectMqtt() {
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  if (mqtt.connected()) return true;

  String cid = "esp32panel-mrsrfbtn-";
  cid += String((uint32_t)ESP.getEfuseMac(), HEX);

  if (!mqtt.connect(cid.c_str())) return false;

  // monitor
  mqtt.subscribe(T_OP_TEMP);
  mqtt.subscribe(T_OP_CPU);
  mqtt.subscribe(T_OP_RAM);

  mqtt.subscribe(T_MP_TEMP);
  mqtt.subscribe(T_MP_CPU);
  mqtt.subscribe(T_MP_RAM);

  // brightness
  mqtt.subscribe(T_BRIGHT_SET);

  // fan pwm
  mqtt.subscribe(T_FAN_SET);

  // relays
  mqtt.subscribe(T_RELAY1_SET);
  mqtt.subscribe(T_RELAY2_SET);
  mqtt.subscribe(T_RELAY3_SET);
  mqtt.subscribe(T_RELAY4_SET);

  // speaker
  mqtt.subscribe(T_SPK_PLAY);

  // RF433
  mqtt.subscribe(TOPIC_TX);
  mqtt.subscribe(TOPIC_HOLD97);
  rfPublishOnline();
  rfLastStatusMs = millis();

  publishBrightnessState();
  publishRelayState(0);
  publishRelayState(1);
  publishRelayState(2);
  publishRelayState(3);
  publishFanState();

  // DS18B20 discovery
  psuDiscoverySent = false;
  psuPublishDiscovery();

  return true;
}

// =====================================================
// ================== SETUP / LOOP ======================
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  // TFT
  tft.init();
  tft.setRotation(ROT);
  tft.setTextFont(2);
  tft.fillScreen(BG);
  buildLayout();
  drawTopStatic();
  updateTopDynamic(true);
  prev = cur;

  // brightness
  applyBrightness(brightMode);

  // fan pwm
  pinMode(FAN_PWM_PIN, OUTPUT);
  bool pwmOk = ledcAttach(FAN_PWM_PIN, PWM_FREQ, PWM_RESOLUTION);
  if (!pwmOk) {
    Serial.println("FAN PWM ATTACH FAILED");
  } else {
    Serial.println("FAN PWM ATTACH OK");
    applySpeedLevel(7);
  }

  // relays init (active LOW)
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  relaySet(0, false);
  relaySet(1, false);
  relaySet(2, false);
  relaySet(3, false);

  // speaker
  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);
  spkStop();

  // ADC
  analogReadResolution(12);
  pinMode(PIN_A, INPUT);
  pinMode(PIN_B, INPUT);
  pinMode(PIN_C, INPUT);
  analogSetPinAttenuation(PIN_C, ADC_11db);

  // RF433
  rf.enableReceive(digitalPinToInterrupt(RF_RX_PIN));
  rf.enableTransmit(RF_TX_PIN);
  rf.setProtocol(RF_PROTOCOL);
  rf.setPulseLength(RF_PULSE);
  rf.setRepeatTransmit(8);

  // DS18B20
  psuSensors.begin();
  psuNextReadMs = millis() + 500;

  // MQTT
  mqtt.setBufferSize(512);
  connectWifi();

  Serial.println("READY");
}

void loop() {

  // ================= WIFI =================
  bool wifiNow = (WiFi.status() == WL_CONNECTED);
  if (wifiNow != cur.wifiOk) {
    cur.wifiOk = wifiNow;
    dirty = true;
  }

  if (!wifiNow) {
    static uint32_t lastTry = 0;
    if (millis() - lastTry > 3000) {
      lastTry = millis();
      connectWifi();
    }
  } else {
    connectMqtt();
    mqtt.loop();
    psuPublishDiscovery();
  }

  // ================= HOLD97 =================
  if (hold97Active) {
    uint32_t now = millis();
    if ((int32_t)(now - hold97UntilMs) >= 0) {
      hold97Active = false;
    } else {
      if (hold97NextSendMs == 0 || (int32_t)(now - hold97NextSendMs) >= 0) {
        rf.send(HOLD97_CODE, RF_BITS);
        hold97NextSendMs = now + HOLD97_INTERVAL_MS;
      }
    }
  }

  // ================= SPEAKER =================
  spkTick();

  // ================= BUTTONS =================
  buttonsTick();

  // ================= PSU TEMP =================
  psuTick();

  // ================= RELAY PULSE =================
  uint32_t now = millis();
  for (int i = 0; i < 4; i++) {
    if (relayPulseActive[i] && (int32_t)(now - relayPulseOffAt[i]) >= 0) {
      relayPulseActive[i] = false;
      relaySet(i, false);
    }
  }

  // ================= RF KEEPALIVE =================
  if (mqtt.connected() && (millis() - rfLastStatusMs > 30000)) {
    rfPublishOnline();
    rfLastStatusMs = millis();
  }

  // ================= RF RECEIVE -> MQTT =================
  if (rf.available()) {
    uint32_t value = rf.getReceivedValue();
    int bits  = rf.getReceivedBitlength();
    int proto = rf.getReceivedProtocol();
    int pulse = rf.getReceivedDelay();
    rf.resetAvailable();

    if (value != 0 && bits == RF_BITS && proto == RF_PROTOCOL) {
      bool isTrigger = inList(value, TRIGGERS, TRIGGERS_N);

      if (mqtt.connected()) {

        // триггеры (8 кодов) блокируем на TRIGGER_BLOCK_MS
        if (isTrigger) {
          uint32_t nowT = millis();

          if (nowT - lastTriggerMs >= TRIGGER_BLOCK_MS) {
            lastTriggerMs = nowT;

            mqtt.publish(TOPIC_LAST, String(value).c_str(), false);

            String raw = "{\"value\":";
            raw += value;
            raw += ",\"bits\":";
            raw += bits;
            raw += ",\"proto\":";
            raw += proto;
            raw += ",\"pulse\":";
            raw += pulse;
            raw += ",\"is_trigger\":true}";
            mqtt.publish(TOPIC_LAST_RAW, raw.c_str(), false);

            mqtt.publish("rf433/rx/trigger", String(value).c_str(), false);
          }
        }
        else if (inList(value, LIGHTS, LIGHTS_N)) {
          mqtt.publish(TOPIC_LAST, String(value).c_str(), false);

          String raw = "{\"value\":";
          raw += value;
          raw += ",\"bits\":";
          raw += bits;
          raw += ",\"proto\":";
          raw += proto;
          raw += ",\"pulse\":";
          raw += pulse;
          raw += ",\"is_trigger\":false}";
          mqtt.publish(TOPIC_LAST_RAW, raw.c_str(), false);

          mqtt.publish("rf433/rx/lights", String(value).c_str(), false);
        }
        else {
          mqtt.publish(TOPIC_LAST, String(value).c_str(), false);

          String raw = "{\"value\":";
          raw += value;
          raw += ",\"bits\":";
          raw += bits;
          raw += ",\"proto\":";
          raw += proto;
          raw += ",\"pulse\":";
          raw += pulse;
          raw += ",\"is_trigger\":false}";
          mqtt.publish(TOPIC_LAST_RAW, raw.c_str(), false);

          mqtt.publish("rf433/rx/other", String(value).c_str(), false);
        }
      }
    }
  }

  // ================= ONLINE TIMEOUT =================
  bool opOnlineNow = (lastOPDataMs != 0) && (now - lastOPDataMs <= DATA_TIMEOUT_MS);
  if (opOnlineNow != cur.a.online) {
    cur.a.online = opOnlineNow;
    if (!opOnlineNow) resetServerData(cur.a);
    dirty = true;
  }

  bool mpOnlineNow = (lastMPDataMs != 0) && (now - lastMPDataMs <= DATA_TIMEOUT_MS);
  if (mpOnlineNow != cur.b.online) {
    cur.b.online = mpOnlineNow;
    if (!mpOnlineNow) resetServerData(cur.b);
    dirty = true;
  }

  bool haNow = (lastAnyDataMs != 0) && (now - lastAnyDataMs <= DATA_TIMEOUT_MS);
  if (haNow != cur.haOk) {
    cur.haOk = haNow;
    dirty = true;
  }

  // ================= UI UPDATE =================
  if (dirty || (now - lastUiTick > 1500)) {
    updateTopDynamic(false);
    prev = cur;
    dirty = false;
    lastUiTick = now;
  }

  delay(5);
}

