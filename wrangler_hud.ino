#include <TFT_eSPI.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <TinyGPSPlus.h>

// ================= DISPLAY =================
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite hud = TFT_eSprite(&tft);

// ================= SENSORS =================
Adafruit_LSM6DSOX imu;
TinyGPSPlus gps;
#define gpsSerial Serial1

// ================= ENCODER =================
#define ENC_CLK 2
#define ENC_DT  3
#define ENC_SW  6

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

  tft.init();
  tft.setRotation(1);

  // Sprite covers the full display (320 × 480 landscape = 480 × 320)
  hud.createSprite(480, 320);

  Wire.begin();
  if (!imu.begin_I2C()) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_RED);
    tft.setTextSize(2);
    tft.setCursor(40, 150);
    tft.print("IMU NOT FOUND");
    while (1) delay(100);
  }

  gpsSerial.begin(9600);

  pinMode(ENC_CLK, INPUT);
  pinMode(ENC_DT,  INPUT);
  pinMode(ENC_SW,  INPUT_PULLUP);

  lastCLK   = digitalRead(ENC_CLK);
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

  // ---- Heading from gyro (dt-corrected, degrees/s → degrees) ----
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
  hud.fillSprite(TFT_BLACK);

  float target = (float)(menuIndex * 30);
  float delta  = target - menuOffsetF;
  menuOffsetF += delta * 0.3f;
  if (fabsf(delta) < 1.0f) menuOffsetF = target;  // snap when close

  for (int i = 0; i < menuSize; i++) {
    int y = 50 + (i * 30) - (int)menuOffsetF;
    hud.setTextColor(i == menuIndex ? TFT_YELLOW : TFT_WHITE);
    drawIcon(40, y + 10, i);
    hud.setCursor(70, y);
    hud.print(menuItems[i]);
  }

  hud.pushSprite(0, 0);
}

void drawIcon(int x, int y, int type) {
  switch (type) {
    case 0: hud.drawLine(x-5, y, x+5, y, TFT_WHITE);          break;
    case 1: hud.drawCircle(x, y, 4, TFT_GREEN);               break;
    case 2: hud.drawCircle(x, y, 5, TFT_WHITE);               break;
    case 3: hud.drawFastHLine(x-5, y, 10, TFT_YELLOW);        break;
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
  // Simple fade to black – fast enough to feel snappy, no long block
  for (int i = 0; i < 480; i += 40) {
    tft.fillRect(i, 0, 40, 320, TFT_BLACK);
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
  const int cx = 240, cy = 160;  // center of 480x320 landscape display

  float r = (roll  - rollOffset)  * DEG_TO_RAD;
  int   p = (pitch - pitchOffset) * 3;

  const int len = 500;
  int x1 = cx - len * cosf(r);
  int y1 = cy - len * sinf(r) + p;
  int x2 = cx + len * cosf(r);
  int y2 = cy + len * sinf(r) + p;

  // Fill sky
  tft.fillRect(0, 0, 480, 320, TFT_NAVY);

  // Draw ground using the horizon line equation
  for (int y = 0; y < 320; y += 3) {
    int denom = (y2 - y1);
    if (denom == 0) denom = 1;
    int x = (y - y1) * (x2 - x1) / denom + x1;
    if (y > y1)
      tft.drawFastHLine(x,     y, 480 - x, TFT_BROWN);
    else
      tft.drawFastHLine(0,     y, x,        0x3400);   // dark green
  }

  tft.drawLine(x1, y1, x2, y2, TFT_WHITE);
  drawJeep(240, 296);  // bottom-center: wheel base lands at y=320
}

// ================= JEEP =================
void drawJeep(int cx, int cy) {
  int x = cx - 35, y = cy - 25;
  tft.fillRect(x + 10, y + 18, 60, 20, TFT_YELLOW);
  tft.fillRect(x + 20, y +  8, 30, 10, TFT_YELLOW);
  tft.fillCircle(x + 20, y + 40, 9, TFT_BLACK);
  tft.fillCircle(x + 60, y + 40, 9, TFT_BLACK);
}

// ================= OVERLAY =================
void drawOverlay() {
  hud.fillSprite(TFT_TRANSPARENT);
  hud.setTextSize(2);

  // Heading
  hud.setTextColor(TFT_WHITE);
  hud.setCursor(10, 10);
  hud.print("HDG:");
  hud.print((int)(heading - headingOffset + 360) % 360);

  // Speed / GPS status
  hud.setCursor(150, 10);
  if (gpsFix) {
    hud.setTextColor(TFT_GREEN);
    hud.print(speed_kmh, 1);
    hud.print("k");
  } else {
    hud.setTextColor(TFT_RED);
    hud.print("NO GPS");
  }

  // Roll / Pitch
  hud.setTextColor(TFT_WHITE);
  hud.setCursor(10,  50); hud.print("R:"); hud.print(roll  - rollOffset,  1);
  hud.setCursor(150, 50); hud.print("P:"); hud.print(pitch - pitchOffset, 1);

  // Danger warning
  if (fabsf(roll - rollOffset) > 35.0f || fabsf(pitch - pitchOffset) > 35.0f) {
    hud.setTextColor(TFT_RED);
    hud.setTextSize(3);
    hud.setCursor(80, 20);
    hud.print("DANGER");
  }

  hud.pushSprite(0, 0, TFT_TRANSPARENT);
}

// ================= GPS SCREEN =================
void drawGPSScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);

  if (!gpsFix) {
    tft.setCursor(80, 220);
    tft.setTextColor(TFT_RED);
    tft.print("NO GPS FIX");
    return;
  }

  tft.setCursor(20, 80);  tft.print("SPD:"); tft.print(speed_kmh, 1); tft.print(" km/h");
  tft.setCursor(20, 120); tft.print("LAT:"); tft.print(gps.location.lat(), 5);
  tft.setCursor(20, 160); tft.print("LON:"); tft.print(gps.location.lng(), 5);
  tft.setCursor(20, 200); tft.print("SAT:"); tft.print(gps.satellites.value());
  tft.setCursor(20, 240); tft.print("ALT:"); tft.print(gps.altitude.meters(), 0); tft.print("m");
}

// ================= INFO SCREEN =================
void drawInfoScreen() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);

  tft.setCursor(40, 100); tft.print("Pitch:  "); tft.print(pitch - pitchOffset, 1);
  tft.setCursor(40, 140); tft.print("Roll:   "); tft.print(roll  - rollOffset,  1);
  tft.setCursor(40, 180); tft.print("Heading:"); tft.print((int)(heading - headingOffset + 360) % 360);
  tft.setCursor(40, 220); tft.print("P-off:  "); tft.print(pitchOffset,   1);
  tft.setCursor(40, 260); tft.print("R-off:  "); tft.print(rollOffset,    1);
}

// ================= BOOT =================
void showBoot() {
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(3);
  tft.setTextColor(TFT_GREEN);
  tft.setCursor(60, 120); tft.print("WRANGLER");
  tft.setCursor(40, 160); tft.print("SYSTEM");
  delay(2000);
}
