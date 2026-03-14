#include <Wire.h>
#include <HX711.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_WIDTH    128
#define OLED_HEIGHT   64
#define OLED_ADDR     0x3C
#define OLED_RESET    -1

Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire1, OLED_RESET);

#define PIN_GAS         A0
#define PIN_CLUTCH      A1
#define PIN_HX_DT       3
#define PIN_HX_SCK      4
#define PIN_CAL_BUTTON  7

struct PedalCal  { int minVal, maxVal; };
struct SteerCal  { uint16_t minVal, maxVal; };
struct BrakeCal  { long maxForce; };

struct ShifterCal {
  uint16_t xMin, xMax;
  uint16_t yMin, yMax;
  uint16_t gearX[6];
  uint16_t gearY[6];
};

PedalCal   gasCal, clutchCal;
SteerCal   steerCal;
BrakeCal   brakeCal;
ShifterCal shiftCal;

#if !defined(USB_KEYBOARDJOYSTICK) && !defined(USB_HID) && !defined(USB_JOYSTICK)
  #error "Set Tools > USB Type > Keyboard + Mouse + Joystick before compiling!"
#endif

HX711 brake;

#define TCA_ADDR       0x70
#define AS5600_ADDR    0x36
#define TCA_CH_STEER   0
#define TCA_CH_SHIFT_X 1
#define TCA_CH_SHIFT_Y 2

void tcaSelect(uint8_t ch) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  Wire.endTransmission();
}

void tcaNone() {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(0);
  Wire.endTransmission();
}

uint16_t readAS5600() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(0x0C);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2);
  if (Wire.available() >= 2) {
    uint16_t h = Wire.read();
    uint16_t l = Wire.read();
    return ((h << 8) | l) & 0x0FFF;
  }
  return 2048;
}

uint16_t readSteering() { tcaSelect(TCA_CH_STEER);   return readAS5600(); }
uint16_t readShiftX()   { tcaSelect(TCA_CH_SHIFT_X); return readAS5600(); }
uint16_t readShiftY()   { tcaSelect(TCA_CH_SHIFT_Y); return readAS5600(); }

#define EEPROM_MAGIC       0
#define EEPROM_GAS_MIN     2
#define EEPROM_GAS_MAX     6
#define EEPROM_CLUTCH_MIN  10
#define EEPROM_CLUTCH_MAX  14
#define EEPROM_BRAKE_MAX   26
#define EEPROM_STEER_MIN   30
#define EEPROM_STEER_MAX   34
#define EEPROM_SHIFTX_MIN  38
#define EEPROM_SHIFTX_MAX  42
#define EEPROM_SHIFTY_MIN  46
#define EEPROM_SHIFTY_MAX  50
#define EEPROM_GEAR_BASE   54
#define EEPROM_MAGIC_VAL   0xA55A

template<typename T> void eepromWrite(int addr, T val) { EEPROM.put(addr, val); }
template<typename T> T   eepromRead(int addr)          { T v; EEPROM.get(addr, v); return v; }

bool eepromValid() { return eepromRead<uint16_t>(EEPROM_MAGIC) == EEPROM_MAGIC_VAL; }

void loadCalibration() {
  gasCal.minVal     = eepromRead<int>(EEPROM_GAS_MIN);
  gasCal.maxVal     = eepromRead<int>(EEPROM_GAS_MAX);
  clutchCal.minVal  = eepromRead<int>(EEPROM_CLUTCH_MIN);
  clutchCal.maxVal  = eepromRead<int>(EEPROM_CLUTCH_MAX);
  brakeCal.maxForce = eepromRead<long>(EEPROM_BRAKE_MAX);
  steerCal.minVal   = eepromRead<uint16_t>(EEPROM_STEER_MIN);
  steerCal.maxVal   = eepromRead<uint16_t>(EEPROM_STEER_MAX);
  shiftCal.xMin     = eepromRead<uint16_t>(EEPROM_SHIFTX_MIN);
  shiftCal.xMax     = eepromRead<uint16_t>(EEPROM_SHIFTX_MAX);
  shiftCal.yMin     = eepromRead<uint16_t>(EEPROM_SHIFTY_MIN);
  shiftCal.yMax     = eepromRead<uint16_t>(EEPROM_SHIFTY_MAX);
  for (int i = 0; i < 6; i++) {
    shiftCal.gearX[i] = eepromRead<uint16_t>(EEPROM_GEAR_BASE + i * 4);
    shiftCal.gearY[i] = eepromRead<uint16_t>(EEPROM_GEAR_BASE + i * 4 + 2);
  }
}

void saveCalibration() {
  eepromWrite(EEPROM_MAGIC,      (uint16_t)EEPROM_MAGIC_VAL);
  eepromWrite(EEPROM_GAS_MIN,    gasCal.minVal);
  eepromWrite(EEPROM_GAS_MAX,    gasCal.maxVal);
  eepromWrite(EEPROM_CLUTCH_MIN, clutchCal.minVal);
  eepromWrite(EEPROM_CLUTCH_MAX, clutchCal.maxVal);
  eepromWrite(EEPROM_BRAKE_MAX,  brakeCal.maxForce);
  eepromWrite(EEPROM_STEER_MIN,  steerCal.minVal);
  eepromWrite(EEPROM_STEER_MAX,  steerCal.maxVal);
  eepromWrite(EEPROM_SHIFTX_MIN, shiftCal.xMin);
  eepromWrite(EEPROM_SHIFTX_MAX, shiftCal.xMax);
  eepromWrite(EEPROM_SHIFTY_MIN, shiftCal.yMin);
  eepromWrite(EEPROM_SHIFTY_MAX, shiftCal.yMax);
  for (int i = 0; i < 6; i++) {
    eepromWrite(EEPROM_GEAR_BASE + i * 4,     shiftCal.gearX[i]);
    eepromWrite(EEPROM_GEAR_BASE + i * 4 + 2, shiftCal.gearY[i]);
  }
}

void applyDefaults() {
  gasCal    = {50, 970};
  clutchCal = {50, 970};
  brakeCal.maxForce = 50000L;
  steerCal  = {200, 3895};
  shiftCal.xMin = 100; shiftCal.xMax = 3995;
  shiftCal.yMin = 100; shiftCal.yMax = 3995;
  shiftCal.gearX[0] = 800;  shiftCal.gearY[0] = 3000;
  shiftCal.gearX[1] = 800;  shiftCal.gearY[1] = 1000;
  shiftCal.gearX[2] = 2048; shiftCal.gearY[2] = 3000;
  shiftCal.gearX[3] = 2048; shiftCal.gearY[3] = 1000;
  shiftCal.gearX[4] = 3200; shiftCal.gearY[4] = 3000;
  shiftCal.gearX[5] = 3200; shiftCal.gearY[5] = 1000;
}

#define FILTER_SIZE 8

class RollingAvg {
  int buf[FILTER_SIZE] = {};
  int idx = 0;
  long sum = 0;
  int count = 0;
public:
  int update(int val) {
    sum -= buf[idx];
    buf[idx] = val;
    sum += val;
    idx = (idx + 1) % FILTER_SIZE;
    if (count < FILTER_SIZE) count++;
    return sum / count;
  }
};

RollingAvg gasFilter, clutchFilter;

#define BRAKE_EXPONENT 2.0f

int scaleBrake(long raw) {
  float n = constrain((float)raw / brakeCal.maxForce, 0.0f, 1.0f);
  return (int)(pow(n, BRAKE_EXPONENT) * 1023.0f);
}

void oledClear() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.cp437(true);
}

void oledPrompt(const char* action, const char* instruction) {
  oledClear();
  oled.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(2, 2);
  oled.print("SIM RIG  CALIBRATION");
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(2);
  int16_t x1, y1; uint16_t w, h;
  oled.getTextBounds(action, 0, 0, &x1, &y1, &w, &h);
  oled.setCursor((128 - w) / 2, 18);
  oled.print(action);
  oled.setTextSize(1);
  oled.setCursor(0, 50);
  oled.print(instruction);
  oled.display();
}

void oledBig(const char* top, const char* symbol, const char* bottom) {
  oledClear();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.print(top);
  oled.setTextSize(3);
  int16_t x1, y1; uint16_t w, h;
  oled.getTextBounds(symbol, 0, 0, &x1, &y1, &w, &h);
  oled.setCursor((128 - w) / 2, 20);
  oled.print(symbol);
  oled.setTextSize(1);
  oled.setCursor(0, 56);
  oled.print(bottom);
  oled.display();
}

void oledCountdown(const char* label, int durationMs) {
  int steps = 20;
  int stepMs = durationMs / steps;
  for (int i = 0; i <= steps; i++) {
    oledClear();
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print(label);
    oled.setCursor(0, 14);
    oled.print("Hold still...");
    oled.drawRect(0, 48, 128, 12, SSD1306_WHITE);
    oled.fillRect(2, 50, (int)(124.0f * i / steps), 8, SSD1306_WHITE);
    oled.display();
    delay(stepMs);
  }
}

void oledSaved(const char* what) {
  oledClear();
  oled.setTextSize(1);
  oled.setCursor(20, 10);
  oled.print("SAVED");
  oled.setTextSize(2);
  oled.setCursor(0, 28);
  oled.print(what);
  oled.display();
  delay(600);
}

void waitButton(const char* action, const char* instruction) {
  oledPrompt(action, instruction);
  while (digitalRead(PIN_CAL_BUTTON) == LOW) delay(10);
  while (digitalRead(PIN_CAL_BUTTON) == HIGH) delay(10);
  delay(50);
  while (digitalRead(PIN_CAL_BUTTON) == LOW) delay(10);
}

void calPedal(const char* name, const char* symbol, int pin, PedalCal &cal) {
  oledBig(name, symbol, "RELEASE fully, press BTN");
  while (digitalRead(PIN_CAL_BUTTON) == HIGH) delay(10);
  delay(50);
  while (digitalRead(PIN_CAL_BUTTON) == LOW) delay(10);
  oledCountdown(name, 800);
  cal.minVal = analogRead(pin);
  oledSaved("MIN OK");

  oledBig(name, symbol, "PRESS FULL, press BTN");
  while (digitalRead(PIN_CAL_BUTTON) == HIGH) delay(10);
  delay(50);
  while (digitalRead(PIN_CAL_BUTTON) == LOW) delay(10);
  oledCountdown(name, 800);
  cal.maxVal = analogRead(pin);
  oledSaved("MAX OK");
}

void calBrake() {
  oledBig("BRAKE", "---", "Do NOT touch. Taring...");
  delay(1000);
  brake.tare();
  oledSaved("TARE OK");

  oledBig("BRAKE", "|||", "STOMP IT, press BTN");
  while (digitalRead(PIN_CAL_BUTTON) == HIGH) delay(10);
  delay(50);
  while (digitalRead(PIN_CAL_BUTTON) == LOW) delay(10);
  oledCountdown("BRAKE MAX FORCE", 800);
  brakeCal.maxForce = abs(brake.read());
  oledSaved("FORCE OK");
}

void calSteering() {
  oledBig("STEER", "<<<", "FULL LEFT, press BTN");
  while (digitalRead(PIN_CAL_BUTTON) == HIGH) delay(10);
  delay(50);
  while (digitalRead(PIN_CAL_BUTTON) == LOW) delay(10);
  oledCountdown("LEFT LOCK", 800);
  steerCal.minVal = readSteering();
  oledSaved("LEFT OK");

  oledBig("STEER", ">>>", "FULL RIGHT, press BTN");
  while (digitalRead(PIN_CAL_BUTTON) == HIGH) delay(10);
  delay(50);
  while (digitalRead(PIN_CAL_BUTTON) == LOW) delay(10);
  oledCountdown("RIGHT LOCK", 800);
  steerCal.maxVal = readSteering();
  oledSaved("RIGHT OK");
}

void calShifter() {
  shiftCal.xMin = 4095; shiftCal.xMax = 0;
  shiftCal.yMin = 4095; shiftCal.yMax = 0;

  const char* gearSymbols[] = {"1", "2", "3", "4", "5", "6"};
  const char* gearLabels[]  = {
    "Engage 1st, press BTN",
    "Engage 2nd, press BTN",
    "Engage 3rd, press BTN",
    "Engage 4th, press BTN",
    "Engage 5th, press BTN",
    "Engage 6th, press BTN"
  };

  for (int g = 0; g < 6; g++) {
    oledBig("SHIFTER", gearSymbols[g], gearLabels[g]);
    while (digitalRead(PIN_CAL_BUTTON) == HIGH) delay(10);
    delay(50);
    while (digitalRead(PIN_CAL_BUTTON) == LOW) delay(10);
    oledCountdown("HOLD GEAR", 600);

    uint16_t x = readShiftX();
    uint16_t y = readShiftY();
    shiftCal.gearX[g] = x;
    shiftCal.gearY[g] = y;
    if (x < shiftCal.xMin) shiftCal.xMin = x;
    if (x > shiftCal.xMax) shiftCal.xMax = x;
    if (y < shiftCal.yMin) shiftCal.yMin = y;
    if (y > shiftCal.yMax) shiftCal.yMax = y;

    char buf[12];
    snprintf(buf, sizeof(buf), "GEAR %d OK", g + 1);
    oledSaved(buf);
  }

  tcaNone();
}

void runCalibration() {
  oledClear();
  oled.setTextSize(1);
  oled.setCursor(10, 5);
  oled.print("SIM RIG v4.0");
  oled.setCursor(5, 20);
  oled.print("CALIBRATION MODE");
  oled.setCursor(0, 38);
  oled.print("Press BTN to start");
  oled.setCursor(0, 50);
  oled.print("each step when ready");
  oled.display();
  delay(2000);

  calPedal("GAS",    "G", PIN_GAS,    gasCal);
  calPedal("CLUTCH", "C", PIN_CLUTCH, clutchCal);
  calBrake();
  calSteering();
  calShifter();

  saveCalibration();

  oledClear();
  oled.setTextSize(2);
  oled.setCursor(10, 10);
  oled.print("ALL DONE");
  oled.setTextSize(1);
  oled.setCursor(5, 40);
  oled.print("Saved. Restarting...");
  oled.display();
  delay(2000);

  SCB_AIRCR = 0x05FA0004;
}

int detectGear(uint16_t x, uint16_t y) {
  uint16_t xMid      = (shiftCal.xMin + shiftCal.xMax) / 2;
  uint16_t yMid      = (shiftCal.yMin + shiftCal.yMax) / 2;
  uint16_t xDeadzone = (shiftCal.xMax - shiftCal.xMin) / 8;
  uint16_t yDeadzone = (shiftCal.yMax - shiftCal.yMin) / 6;

  if (abs((int)x - (int)xMid) < xDeadzone &&
      abs((int)y - (int)yMid) < yDeadzone)
    return 0;

  int      bestGear = 0;
  uint32_t bestDist = UINT32_MAX;

  for (int g = 0; g < 6; g++) {
    int dx = (int)x - (int)shiftCal.gearX[g];
    int dy = (int)y - (int)shiftCal.gearY[g];
    uint32_t dist = (uint32_t)(dx*dx) + (uint32_t)(dy*dy);
    if (dist < bestDist) {
      bestDist = dist;
      bestGear = g + 1;
    }
  }

  uint32_t maxAllowed = (uint32_t)(shiftCal.xMax - shiftCal.xMin) * 300;
  return (bestDist < maxAllowed) ? bestGear : 0;
}

int lastDisplayGear = -1;

void updateDisplay(int gear, bool reverse) {
  int displayGear = reverse ? -1 : gear;
  if (displayGear == lastDisplayGear) return;
  lastDisplayGear = displayGear;

  oledClear();

  oled.fillRect(0, 0, 128, 11, SSD1306_WHITE);
  oled.setTextColor(SSD1306_BLACK);
  oled.setTextSize(1);
  oled.setCursor(35, 2);
  oled.print("SIM RIG");

  oled.setTextColor(SSD1306_WHITE);

  if (reverse) {
    oled.setTextSize(4);
    oled.setCursor(44, 16);
    oled.print("R");
  } else if (gear == 0) {
    oled.setTextSize(4);
    oled.setCursor(44, 16);
    oled.print("N");
  } else {
    oled.setTextSize(4);
    char buf[3];
    snprintf(buf, sizeof(buf), "%d", gear);
    int16_t x1, y1; uint16_t w, h;
    oled.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
    oled.setCursor((128 - w) / 2, 16);
    oled.print(buf);
  }

  oled.drawLine(0, 55, 128, 55, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, 57);
  oled.print("GEAR");
  oled.setCursor(90, 57);
  oled.print("v4.0");

  oled.display();
}

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);

  Wire1.begin();
  Wire1.setClock(400000);

  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED not found");
  } else {
    oledClear();
    oled.setTextSize(2);
    oled.setCursor(10, 8);
    oled.print("SIM RIG");
    oled.setTextSize(1);
    oled.setCursor(35, 30);
    oled.print("v4.0");
    oled.setCursor(10, 48);
    oled.print("Teensy 4.1");
    oled.display();
    delay(1500);
  }

  pinMode(PIN_CAL_BUTTON, INPUT_PULLUP);

  brake.begin(PIN_HX_DT, PIN_HX_SCK);
  brake.set_gain(128);

  if (eepromValid()) {
    loadCalibration();
    oledClear();
    oled.setTextSize(1);
    oled.setCursor(10, 20);
    oled.print("Cal loaded OK");
    oled.setCursor(5, 36);
    oled.print("Hold BTN to recal");
    oled.display();
    delay(1200);
  } else {
    applyDefaults();
    oledClear();
    oled.setTextSize(1);
    oled.setCursor(0, 10);
    oled.print("No calibration found");
    oled.setCursor(0, 26);
    oled.print("Hold BTN at boot to");
    oled.setCursor(0, 38);
    oled.print("run calibration");
    oled.display();
    delay(2000);
  }

  delay(200);
  if (digitalRead(PIN_CAL_BUTTON) == LOW) {
    delay(500);
    if (digitalRead(PIN_CAL_BUTTON) == LOW) {
      runCalibration();
    }
  }

  brake.tare();
  tcaNone();

  Joystick.useManualSend(true);

  updateDisplay(0, false);
}

void loop() {

  int gas  = gasFilter.update(analogRead(PIN_GAS));
  int clut = clutchFilter.update(analogRead(PIN_CLUTCH));

  gas  = constrain(map(gas,  gasCal.minVal,    gasCal.maxVal,    0, 1023), 0, 1023);
  clut = constrain(map(clut, clutchCal.minVal, clutchCal.maxVal, 0, 1023), 0, 1023);

  int brakeOut = 0;
  if (brake.is_ready()) {
    brakeOut = scaleBrake(abs(brake.read()));
  }

  uint16_t steerRaw = readSteering();
  int steerOut = constrain(
    map(steerRaw, steerCal.minVal, steerCal.maxVal, -32767, 32767),
    -32767, 32767
  );

  uint16_t shiftX = readShiftX();
  uint16_t shiftY = readShiftY();
  tcaNone();

  int  gear    = detectGear(shiftX, shiftY);
  bool reverse = (shiftY > shiftCal.yMax - 100);

  updateDisplay(gear, reverse);

  int steerSend = map(steerOut, -32767, 32767, 0, 1023);
  Joystick.X(steerSend);
  Joystick.Y(gas);
  Joystick.Z(brakeOut);
  Joystick.Zrotate(clut);

  for (int i = 0; i < 6; i++)
    Joystick.button(i + 1, gear == (i + 1));

  Joystick.button(7, reverse);

  Joystick.send_now();

  delay(4);
}
