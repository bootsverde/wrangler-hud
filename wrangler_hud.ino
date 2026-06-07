#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <TinyGPSPlus.h>

// ================= DISPLAY =================
// Waveshare ESP32-S3-Touch-LCD-4.3 — 800x480 RGB parallel panel.
// Pin map per Waveshare's official Arduino reference for this board.
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    5 /* DE */, 3 /* VSYNC */, 46 /* HSYNC */, 7 /* PCLK */,
    1, 2, 42, 41, 40,        /* R0-R4 */
    39, 0, 45, 48, 47, 21,   /* G0-G5 */
    14, 38, 18, 17, 10,      /* B0-B4 */
    0 /* hsync_polarity */, 40 /* hsync_front_porch */, 48 /* hsync_pulse_width */, 88 /* hsync_back_porch */,
    0 /* vsync_polarity */, 13 /* vsync_front_porch */, 3  /* vsync_pulse_width */, 32 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 16000000 /* prefer_speed */);

Arduino_RGB_Display *gfx = new Arduino_RGB_Display(800, 480, bus);

// Color-name shims so the drawing code below can stay close to the original
// (Arduino_GFX's built-in palette doesn't include TFT_eSPI's NAVY/BROWN names).
#define TFT_BLACK  BLACK
#define TFT_WHITE  WHITE
#define TFT_RED    RED
#define TFT_GREEN  GREEN
#define TFT_YELLOW YELLOW
#define TFT_NAVY   0x000F
#define TFT_BROWN  0x8200

// ================= SENSORS =================
Adafruit_LSM6DSOX imu;
TinyGPSPlus gps;
#define gpsSerial Serial1

// Touch/IMU I2C bus pins for this board (shared bus; IMU and GT911 touch
// can coexist as long as their addresses differ).
#define I2C_SDA 8
#define I2C_SCL 9

// ================= ENCODER =================
// NOTE: original pins 2/3 collide with this board's display data lines
// (R1 = GPIO2, VSYNC = GPIO3). Re-routed to GPIO 11/12/13 — confirm these
// are broken out on your board's expansion header before wiring!
#define ENC_CLK 11
#define ENC_DT  12
#define ENC_SW  13

int      lastCLK;
uint32_t lastBtnMs = 0;

// ================= GLOBAL =================
float pitch = 0, roll = 0, heading = 0;
float pitchOffset = 0, rollOffset = 0, headingOffset = 0;
float speed_kmh = 0;
bool  gpsFix = false;

uint32_t lastLoopMs = 0;   // for heading dt

// ================= MENU =================
bool menuActive   = false;
int  menuIndex    = 0;
int  currentScreen = 0;
float menuOffsetF  = 0.0f;  // float for smooth scroll

const char* menuItems[] = { "Horizon", "GPS", "Info", "Calibrate" };
const int   menuSize    = 4;

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(TFT_BLACK);

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!imu.begin_I2C()) {
    gfx->fillScreen(TFT_BLACK);
    gfx->setTextColor(TFT_RED);
    gfx->setTextSize(2);
    gfx->setCursor(40, 220);
    gfx->print("IMU NOT FOUND");
    while (1) delay(100);
  }

  gpsSerial.begin(9600);

  pinMode(ENC_CLK, INPUT);
  pinMode(ENC_DT,  INPUT);
  pinMode(ENC_SW,  INPUT_PULLUP);

  lastCLK    = digitalRead(ENC_CLK);
  lastLoopMs = millis();

  showBoot();
}

// ================= LOOP =================
void loop() {
  uint32_t now = millis();
  float    dt  = (now - lastLoopMs) / 1000.0f;
  lastLoopMs   = now;

  readEncoder();
  readIMU(dt);   // single getEvent, shared between pitch/roll + heading
  readGPS();

  if (menuActive) {
    drawMenu();
  } else {
    switch (currentScreen) {
      case 0: drawHorizon(); drawOverlay(); break;
      case 1: drawGPSScreen();             break;
      case 2: drawInfoScreen();            break;
    }
  }

  delay(30);
}

// ================= IMU (single read) =================
void readIMU(float dt) {
  sensors_event_t accel, gyro, temp;
  imu.getEvent(&accel, &gyro, &temp);

  // ---- Pitch & roll from accelerometer ----
  float newPitch = atan2(accel.acceleration.x, accel.acceleration.z) * 57.2958f;
  float newRoll  = atan2(accel.acceleration.y, accel.acceleration.z) * 57.2958f;

  const float alpha = 0.1f;
  pitch = pitch * (1.0f - alpha) + newPitch * alpha;
  roll  = roll  * (1.0f - alpha) + newRoll  * alpha;

  // ---- Heading from gyro (dt-corrected, degrees/s -> degrees) ----
  heading += gyro.gyro.z * dt * 57.2958f;
  heading  = fmodf(heading, 360.0f);
  if (heading < 0) heading += 360.0f;
}

// ================= GPS =================
void readGPS() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  gpsFix = gps.location.isValid();
  if (gpsFix) speed_kmh = gps.speed.kmph();
}

// ================= ENCODER =================
void readEncoder() {
  int currentCLK = digitalRead(ENC_CLK);

  if (currentCLK != lastCLK && currentCLK == LOW) {
    if (digitalRead(ENC_DT) != currentCLK) menuIndex++;
    else                                    menuIndex--;

    if (menuIndex < 0)         menuIndex = menuSize - 1;
    if (menuIndex >= menuSize) menuIndex = 0;
  }
  lastCLK = currentCLK;

  // Non-blocking debounce
  if (digitalRead(ENC_SW) == LOW) {
    uint32_t now = millis();
    if (now - lastBtnMs > 150) {
      lastBtnMs = now;
      if (menuActive) {
        handleMenu();
        menuActive = false;
      } else {
        menuActive = true;
      }
    }
  }
}

// ================= MENU =================
void drawMenu() {
  gfx->fillScreen(TFT_BLACK);

  float target = (float)(menuIndex * 50);
  float delta  = target - menuOffsetF;
  menuOffsetF += delta * 0.3f;
  if (fabsf(delta) < 1.0f) menuOffsetF = target;  // snap when close

  for (int i = 0; i < menuSize; i++) {
    int y = 80 + (i * 50) - (int)menuOffsetF;
    gfx->setTextSize(2);
    gfx->setTextColor(i == menuIndex ? TFT_YELLOW : TFT_WHITE);
    drawIcon(80, y + 10, i);
    gfx->setCursor(120, y);
    gfx->print(menuItems[i]);
  }
}

void drawIcon(int x, int y, int type) {
  switch (type) {
    case 0: gfx->drawLine(x-5, y, x+5, y, TFT_WHITE);          break;
    case 1: gfx->drawCircle(x, y, 4, TFT_GREEN);               break;
    case 2: gfx->drawCircle(x, y, 5, TFT_WHITE);               break;
    case 3: gfx->drawFastHLine(x-5, y, 10, TFT_YELLOW);        break;
  }
}

void handleMenu() {
  screenTransition();
  switch (menuIndex) {
    case 0: currentScreen = 0; break;
    case 1: currentScreen = 1; break;
    case 2: currentScreen = 2; break;
    case 3: calibrate();       break;
  }
}

// ================= TRANSITION (non-blocking) =================
void screenTransition() {
  // Simple wipe to black across the 800x480 panel.
  for (int i = 0; i < 800; i += 100) {
    gfx->fillRect(i, 0, 100, 480, TFT_BLACK);
    delay(5);
  }
}

// ================= CALIBRATE =================
void calibrate() {
  pitchOffset   = pitch;
  rollOffset    = roll;
  headingOffset = heading;
}

// ================= HORIZON =================
void drawHorizon() {
  const int cx = 400, cy = 240;  // center of 800x480 display

  float r = (roll  - rollOffset)  * DEG_TO_RAD;
  int   p = (pitch - pitchOffset) * 3;

  const int len = 800;
  int x1 = cx - len * cosf(r);
  int y1 = cy - len * sinf(r) + p;
  int x2 = cx + len * cosf(r);
  int y2 = cy + len * sinf(r) + p;

  // Fill sky
  gfx->fillRect(0, 0, 800, 480, TFT_NAVY);

  // Draw ground using the horizon line equation
  for (int y = 0; y < 480; y += 3) {
    int denom = (y2 - y1);
    if (denom == 0) denom = 1;
    int x = (y - y1) * (x2 - x1) / denom + x1;
    if (y > y1)
      gfx->drawFastHLine(x,     y, 800 - x, TFT_BROWN);
    else
      gfx->drawFastHLine(0,     y, x,        0x3400);   // dark green
  }

  gfx->drawLine(x1, y1, x2, y2, TFT_WHITE);
  drawJeep(400, 456);  // bottom-center: wheel base lands at y=480
}

// ================= JEEP =================
void drawJeep(int cx, int cy) {
  int x = cx - 35, y = cy - 25;
  gfx->fillRect(x + 10, y + 18, 60, 20, TFT_YELLOW);
  gfx->fillRect(x + 20, y +  8, 30, 10, TFT_YELLOW);
  gfx->fillCircle(x + 20, y + 40, 9, TFT_BLACK);
  gfx->fillCircle(x + 60, y + 40, 9, TFT_BLACK);
}

// ================= OVERLAY =================
// Drawn directly onto gfx every frame, right after drawHorizon() repaints
// the full screen — so there's no need for a separate transparent sprite.
void drawOverlay() {
  gfx->setTextSize(2);

  // Heading
  gfx->setTextColor(TFT_WHITE);
  gfx->setCursor(20, 20);
  gfx->print("HDG:");
  gfx->print((int)(heading - headingOffset + 360) % 360);

  // Speed / GPS status
  gfx->setCursor(300, 20);
  if (gpsFix) {
    gfx->setTextColor(TFT_GREEN);
    gfx->print(speed_kmh, 1);
    gfx->print("k");
  } else {
    gfx->setTextColor(TFT_RED);
    gfx->print("NO GPS");
  }

  // Roll / Pitch
  gfx->setTextColor(TFT_WHITE);
  gfx->setCursor(20,  70); gfx->print("R:"); gfx->print(roll  - rollOffset,  1);
  gfx->setCursor(300, 70); gfx->print("P:"); gfx->print(pitch - pitchOffset, 1);

  // Danger warning
  if (fabsf(roll - rollOffset) > 35.0f || fabsf(pitch - pitchOffset) > 35.0f) {
    gfx->setTextColor(TFT_RED);
    gfx->setTextSize(3);
    gfx->setCursor(320, 30);
    gfx->print("DANGER");
  }
}

// ================= GPS SCREEN =================
void drawGPSScreen() {
  gfx->fillScreen(TFT_BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(TFT_WHITE);

  if (!gpsFix) {
    gfx->setCursor(320, 230);
    gfx->setTextColor(TFT_RED);
    gfx->print("NO GPS FIX");
    return;
  }

  gfx->setCursor(60, 100); gfx->print("SPD:"); gfx->print(speed_kmh, 1); gfx->print(" km/h");
  gfx->setCursor(60, 160); gfx->print("LAT:"); gfx->print(gps.location.lat(), 5);
  gfx->setCursor(60, 220); gfx->print("LON:"); gfx->print(gps.location.lng(), 5);
  gfx->setCursor(60, 280); gfx->print("SAT:"); gfx->print(gps.satellites.value());
  gfx->setCursor(60, 340); gfx->print("ALT:"); gfx->print(gps.altitude.meters(), 0); gfx->print("m");
}

// ================= INFO SCREEN =================
void drawInfoScreen() {
  gfx->fillScreen(TFT_BLACK);
  gfx->setTextSize(2);
  gfx->setTextColor(TFT_WHITE);

  gfx->setCursor(80, 120); gfx->print("Pitch:  "); gfx->print(pitch - pitchOffset, 1);
  gfx->setCursor(80, 180); gfx->print("Roll:   "); gfx->print(roll  - rollOffset,  1);
  gfx->setCursor(80, 240); gfx->print("Heading:"); gfx->print((int)(heading - headingOffset + 360) % 360);
  gfx->setCursor(80, 300); gfx->print("P-off:  "); gfx->print(pitchOffset,   1);
  gfx->setCursor(80, 360); gfx->print("R-off:  "); gfx->print(rollOffset,    1);
}

// ================= BOOT =================
void showBoot() {
  gfx->fillScreen(TFT_BLACK);
  gfx->setTextSize(4);
  gfx->setTextColor(TFT_GREEN);
  gfx->setCursor(260, 200); gfx->print("WRANGLER");
  gfx->setCursor(310, 260); gfx->print("SYSTEM");
  delay(2000);
}
