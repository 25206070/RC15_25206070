#include <TinyGPS++.h>
#include <SPI.h>
#include <SdFat.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>

// ================= GPS =================
TinyGPSPlus gps;

// ================= SD (SdFat) =================
SdFat SD;
SdFile dataFile;
const int chipSelect = 4;

// Generate a new filename at each startup (instead of the fixed Data.csv)
char filename[24];  // e.g. "250116_093300.csv" or "BOOT0001.csv"

// ================= GSR =================
const int GSRPIN = A0;

// ================= LED (WS2812/NeoPixel) =================
#define LED_PIN    6
#define LED_COUNT  30   // !!! If you cut less than 30 pieces, please change to the actual number (e.g., 5)
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Everything else remains the same, just make the LED more sensitive: apply gain + gamma after rawLevel
const float LED_SMOOTH = 0.15f;   // Smaller values make it more responsive (recommended 0.10~0.20)
const float LED_GAIN   = 1.8f;    // Larger values make it more sensitive (recommended 1.3~2.2)
const float LED_GAMMA  = 0.60f;   // <1 amplifies small fluctuations (recommended 0.50~0.80)
float ledLevel = 0.0f;            // 0~1

// Dynamic range tracking (automatic gain): ensures noticeable changes for different people/environments
float scrAbsEMA   = 5.0f;
float slopeAbsEMA = 2.0f;
const float RANGE_ALPHA = 0.02f;

// ================= Index =================
unsigned long indexCount = 0;

// ================= SCR / Slope / Recovery =================
float gsr_baseline = NAN;
float scr = 0.0f;
float prev_scr = 0.0f;
unsigned long prev_ms = 0;
float slope_cps = 0.0f;

bool in_event = false;
unsigned long recovery_start_ms = 0;
float recovery_time_s = -1.0f;

const float BASELINE_ALPHA = 0.01f;
const float EVENT_ON_TH = 8.0f;
const float EVENT_OFF_TH = 3.0f;
const float RECOVER_BAND = 2.0f;

// ---------- Date/Time helpers (no extra libraries) ----------
static bool isLeap(int y) {
  return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}
static int dim(int y, int m) {
  static const int d[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (m == 2) return d[m-1] + (isLeap(y) ? 1 : 0);
  return d[m-1];
}

// add hours to (y,m,d,hh) with date carry
static void addHours(int &y, int &m, int &dday, int &hh, int addh) {
  hh += addh;
  while (hh >= 24) {
    hh -= 24;
    dday++;
    if (dday > dim(y, m)) {
      dday = 1;
      m++;
      if (m > 12) { m = 1; y++; }
    }
  }
  while (hh < 0) {
    hh += 24;
    dday--;
    if (dday < 1) {
      m--;
      if (m < 1) { m = 12; y--; }
      dday = dim(y, m);
    }
  }
}

// build BOOTxxxx.csv name
static void makeBootFilename(char *out, size_t n) {
  for (int i = 0; i < 10000; i++) {
    snprintf(out, n, "BOOT%04d.csv", i);
    if (!SD.exists(out)) return;
  }
  snprintf(out, n, "BOOTX.csv");
}

// build YYMMDD_HHMMSS.csv from GPS UTC time (if valid)
static bool makeGpsFilenameUTC(char *out, size_t n) {
  if (!(gps.date.isValid() && gps.time.isValid())) return false;

  int y = gps.date.year();    // 4-digit
  int mo = gps.date.month();
  int da = gps.date.day();
  int hh = gps.time.hour();
  int mi = gps.time.minute();
  int ss = gps.time.second();

  int yy = y % 100;
  snprintf(out, n, "%02d%02d%02d_%02d%02d%02d.csv", yy, mo, da, hh, mi, ss);

  // if exists, add small suffix
  if (SD.exists(out)) {
    for (int k = 1; k < 100; k++) {
      snprintf(out, n, "%02d%02d%02d_%02d%02d%02d_%02d.csv", yy, mo, da, hh, mi, ss, k);
      if (!SD.exists(out)) return true;
    }
  }
  return true;
}

// ====== LED color mapping: blue -> green -> yellow -> red, brightness scales with level ======
static uint32_t colorFromLevel(float x, uint8_t brightness) {
  x = constrain(x, 0.0f, 1.0f);

  uint8_t r=0, g=0, b=0;

  if (x < 0.4f) {
    float t = x / 0.4f;          // 0..1
    // blue(0,0,255) -> green(0,255,0)
    r = 0;
    g = (uint8_t)(255 * t);
    b = (uint8_t)(255 * (1.0f - t));
  } else if (x < 0.7f) {
    float t = (x - 0.4f) / 0.3f; // 0..1
    // green(0,255,0) -> yellow(255,255,0)
    r = (uint8_t)(255 * t);
    g = 255;
    b = 0;
  } else {
    float t = (x - 0.7f) / 0.3f; // 0..1
    // yellow(255,255,0) -> red(255,0,0)
    r = 255;
    g = (uint8_t)(255 * (1.0f - t));
    b = 0;
  }

  // apply brightness
  r = (uint8_t)((uint16_t)r * brightness / 255);
  g = (uint8_t)((uint16_t)g * brightness / 255);
  b = (uint8_t)((uint16_t)b * brightness / 255);

  return strip.Color(r, g, b);
}

static void showEmotionLED(float level01) {
  level01 = constrain(level01, 0.0f, 1.0f);
  uint8_t bri = (uint8_t)constrain(30 + (int)(225 * level01), 0, 255);
  uint32_t c = colorFromLevel(level01, bri);

  for (int i = 0; i < LED_COUNT; i++) strip.setPixelColor(i, c);
  strip.show();
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);   // GPS on Serial1

  while (!Serial) {}

  Serial.println("System start");

  // ---------- LED init ----------
  strip.begin();
  strip.show();            // all off
  strip.setBrightness(255);

  // ---------- SD init ----------
  if (!SD.begin(chipSelect, SD_SCK_MHZ(4))) {
    Serial.println("SD init failed");
    SD.initErrorHalt();
  }
  Serial.println("SD init OK");

  // ---------- Generate NEW CSV filename each boot ----------
  // Try briefly to get GPS time for nicer filename
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {           // <= 2s wait for GPS NMEA (does not affect subsequent logic)
    while (Serial1.available()) gps.encode(Serial1.read());
    if (gps.date.isValid() && gps.time.isValid()) break;
  }

  if (!makeGpsFilenameUTC(filename, sizeof(filename))) {
    makeBootFilename(filename, sizeof(filename));
  }

  Serial.print("Log file: ");
  Serial.println(filename);

  // ---------- Write CSV header ----------
  if (!SD.exists(filename)) {
    if (dataFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
      dataFile.println("index,gpsTimeValid,date,time,millis,lat,lng,speed_mph,gsr,scr,slope_cps,recovery_s,it_date,it_time,emo_level");
      dataFile.close();
      Serial.println("CSV header written");
    }
  }

  prev_ms = millis();
}

void loop() {
  // ---------- Read GPS stream ----------
  while (Serial1.available()) {
    gps.encode(Serial1.read());
  }

  // ---------- Read GSR (50-sample average) ----------
  long gsr_sum = 0;
  for (int i = 0; i < 50; i++) {
    gsr_sum += analogRead(GSRPIN);
    delay(5);
  }
  int gsr_average = gsr_sum / 50;

  // ---------- SCR / baseline / slope / recovery ----------
  unsigned long now_ms = millis();
  float dt_s = (now_ms - prev_ms) / 1000.0f;
  if (dt_s <= 0) dt_s = 0.001f;

  if (isnan(gsr_baseline)) {
    gsr_baseline = (float)gsr_average;
  } else {
    gsr_baseline = (1.0f - BASELINE_ALPHA) * gsr_baseline + BASELINE_ALPHA * (float)gsr_average;
  }

  scr = (float)gsr_average - gsr_baseline;
  slope_cps = (scr - prev_scr) / dt_s;

  if (!in_event && scr >= EVENT_ON_TH) {
    in_event = true;
    recovery_time_s = -1.0f;
  }
  if (in_event && scr <= EVENT_OFF_TH) {
    in_event = false;
    recovery_start_ms = now_ms;
    recovery_time_s = 0.0f;
  }

  if (recovery_time_s >= 0.0f) {
    recovery_time_s = (now_ms - recovery_start_ms) / 1000.0f;
    if (fabs(scr) <= RECOVER_BAND) {
      recovery_start_ms = now_ms; // Keep basically unchanged (freeze)
    }
  }

  prev_scr = scr;
  prev_ms = now_ms;

  // ---------- Check GPS time ----------
  bool gpsTimeValid = gps.time.isValid() && gps.date.isValid();

  // ---------- Italy local time (GPS time + 1 hour, NO DST) ----------
  unsigned long it_date = 0;
  unsigned long it_time = 0;

  if (gpsTimeValid) {
    int y  = gps.date.year();
    int mo = gps.date.month();
    int da = gps.date.day();

    int hh = gps.time.hour();
    int mi = gps.time.minute();
    int ss = gps.time.second();
    int cc = gps.time.centisecond();

    addHours(y, mo, da, hh, 1);

    int yy = y % 100;
    it_date = (unsigned long)(yy * 10000UL + mo * 100UL + da);
    it_time = (unsigned long)(hh * 1000000UL + mi * 10000UL + ss * 100UL + cc);
  }

  // ================= LED "emotion" metric (more visible/detectable, everything else unchanged) =================
  float scrAbs   = fabs(scr);
  float slopeAbs = fabs(slope_cps);

  // Dynamic range tracking (keep as original)
  scrAbsEMA   = (1.0f - RANGE_ALPHA) * scrAbsEMA   + RANGE_ALPHA * scrAbs;
  slopeAbsEMA = (1.0f - RANGE_ALPHA) * slopeAbsEMA + RANGE_ALPHA * slopeAbs;

  float scrRef   = max(scrAbsEMA,   1.0f);
  float slopeRef = max(slopeAbsEMA, 0.5f);

  // Normalize (keep as original)
  float scrN   = scrAbs   / (scrRef   * 2.0f);
  float slopeN = slopeAbs / (slopeRef * 2.5f);

  float rawLevel = 0.55f * scrN + 0.45f * slopeN;
  rawLevel = constrain(rawLevel, 0.0f, 1.0f);

  // Sensitivity boost (keep as original)
  rawLevel = constrain(rawLevel * LED_GAIN, 0.0f, 1.0f);
  rawLevel = pow(rawLevel, LED_GAMMA);

  // Key enhancement: minimum visible "floor" + peak hold (LED display only)
  const float LED_FLOOR = 0.06f;  // 0.04~0.10
  const float PEAK_HOLD = 0.12f;  // 0.08~0.20

  static float held = 0.0f;
  held = max(rawLevel, held * (1.0f - PEAK_HOLD));

  // Smooth (keep as original)
  ledLevel = (1.0f - LED_SMOOTH) * ledLevel + LED_SMOOTH * held;

  // Show LED (always visible)
  showEmotionLED(max(ledLevel, LED_FLOOR));

  // ---------- Build CSV line ----------
  String line = "";
  line += String(indexCount); line += ",";
  line += (gpsTimeValid ? "1," : "0,");

  if (gpsTimeValid) {
    line += String(gps.date.value()); line += ",";
    line += String(gps.time.value()); line += ",";
  } else {
    line += "0,0,";
  }

  line += String(now_ms); line += ",";

  line += (gps.location.isValid() ? String(gps.location.lat(), 6) : "0"); line += ",";
  line += (gps.location.isValid() ? String(gps.location.lng(), 6) : "0"); line += ",";
  line += (gps.speed.isValid()    ? String(gps.speed.mph())        : "0"); line += ",";

  line += String(gsr_average); line += ",";
  line += String(scr, 2); line += ",";
  line += String(slope_cps, 2); line += ",";
  line += String(recovery_time_s, 2);

  // Append Italy local time
  line += ",";
  line += String(it_date);
  line += ",";
  line += String(it_time);

  // Append emotion level
  line += ",";
  line += String(ledLevel, 3);

  // Always print to Serial
  Serial.println(line);

 // Keep your current GSR output (may interfere with Serial Plotter parsing multiple columns; if you want to plot curves, it is recommended to comment out)
  Serial.print("GSR ");
  Serial.println(gsr_average);

  // Always write to SD
  if (dataFile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
    dataFile.println(line);
    dataFile.close();
    indexCount++;
  } else {
    Serial.println("SD write failed");
  }
}
