#include <Arduino.h>
#include <EEPROM.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>
#include <LiquidCrystal.h>

/* ============================================================
   WATER PUMP CONTROL SYSTEM - 2 PUMPS
   Board  : Arduino Uno
   Display: D1 Robot LCD Keypad Shield (16x2 LCD + 6 buttons)

   SHIELD PIN USAGE (FIXED by the shield hardware):
     LCD   RS=8  E=9  D4..D7=4,5,6,7  Backlight=10
     Buttons = A0 (analog voltage divider, 6 push buttons)
   RTC DS1302 (3-wire): DAT = A4, CLK = A5, RST/CE = 12

   FREE PINS USED BY THIS SKETCH:
     sensor_well = 2   well / water-source level sensor
     sensor_tank = 3   tank level sensor (external interrupt INT1)
     pump1       = A1  pump 1 (well -> tank)
     pump2       = A2  pump 2
     voltage     = A3  battery/supply voltage (resistive divider)
     buzzer      = 13
     ledAuto     = 11  panel LED, ON in AUTO mode
     rtcIO/rtcCLK/rtcCE = A4/A5/12  (DS1302 DAT, CLK, RST)

   VOLTAGE DIVIDER (from the 12V battery through A3):
      12V+ ----[10k]----+---- A3
                        |
                      [4.7k]
                        |
                       GND
     DIV_FACTOR100 below assumes R1=10k, R2=4.7k.
     For other values use:  DIV_FACTOR100 = round((R1+R2)/R2 * 50)

   FEATURES:
     - Modes: OFF / AUTO (sensors or timed rotation) / MANUAL
     - Protections: dry-run (grace timer), low voltage,
       pumps-never-simultaneously (interlock + fault)
     - Runtime counters per pump (hours) + maintenance reminder
     - Event log (last 15 events with RTC time), kept in EEPROM

   NOTE: EEPROM addresses of the original code are preserved:
     0=mode, 3=control.  Pump times: 60/70/80.
   ============================================================ */

// ---------------- RTC and LCD ----------------
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ---------------- Pin definitions ----------------
const byte sensor_well  = 2;    // well/source level sensor
const byte sensor_tank  = 3;    // tank level sensor (INT1)
const byte pump1        = A1;   // pump 1
const byte pump2        = A2;   // pump 2
const byte pumpPins[2]  = { pump1, pump2 };
const byte PUMP_ON  = LOW;    // relay modules are active-LOW
const byte PUMP_OFF = HIGH;
const byte voltagePin   = A3;   // battery divider (analog input)
const byte ledAuto      = 11;
const byte buzzer       = 13;

// ---------------- RTC DS1302 (3-wire) ----------------
const byte rtcIO  = A4;   // DAT  -> DS1302 I/O
const byte rtcCLK = A5;   // CLK  -> DS1302 SCLK
const byte rtcCE  = 12;   // RST  -> DS1302 CE/RST  (formerly ledManual)
ThreeWire rtcWire(rtcIO, rtcCLK, rtcCE);
RtcDS1302<ThreeWire> rtc(rtcWire);

// ---------------- Voltage divider ----------------
const byte DIV_FACTOR100 = 156;   // -> measured V in 0.1V units (max 15.6V)

// ---------------- Buzzer tones ----------------
const int toneKey    = 1000;   // Hz, normal key press
const int toneSaved  = 2000;   // Hz, value saved
const int durShort   = 50;     // ms
const int durLong    = 200;    // ms

// ---------------- EEPROM addresses ----------------
const byte addrMode        = 0;    // operating mode
const byte addrControl     = 3;    // control type (sensors / time)
const byte addrRunP1       = 12;   // pump1 runtime seconds (unsigned long)
const byte addrRunP2       = 16;   // pump2 runtime seconds (unsigned long)
const byte addrPumpTime[2] = { 60, 70 };
const byte addrDryMin      = 90;   // dry-run grace minutes (0=off)
const byte addrVoltLow     = 91;   // low voltage threshold 9..13V (0=off)
const byte addrMaintHours  = 92;   // maintenance interval hours (uint16_t)
const byte addrLogIdx      = 94;   // event log write index
const byte addrLogFull     = 95;   // event log wrapped flag (1=full)
const byte addrLogBase     = 100;  // event log array (15 x 6 bytes)

// ---------------- Event log ----------------
#define EVT_MAX 15
struct LogEvent {
  byte type;
  byte hh, mm, ss, dd, mo;
};
const byte EVT_START    = 1;
const byte EVT_STOP     = 2;
const byte EVT_DRY      = 3;
const byte EVT_DRY_OK   = 4;
const byte EVT_VOLT     = 5;
const byte EVT_VOLT_OK  = 6;
const byte EVT_BOTH     = 7;
const byte EVT_CLEAR    = 8;
const byte EVT_BOOT     = 9;
const byte EVT_MAINT    = 10;
const char evtNames[10][7] PROGMEM =
  { "START","STOP ","DRY!","WELLOK","VOLT!","VOLTOK","BOTH!","CLEAR","BOOT","MAINT" };

LogEvent evts[EVT_MAX];
byte evtIdx   = 0;     // next write position
byte evtCount = 0;     // stored events (max EVT_MAX)

// ---------------- Global state ----------------
byte mode          = 0;        // 0=OFF, 1=AUTO, 2=MANUAL
bool controlByTime = false;    // false = sensors, true = timed rotation
byte manualPump    = 0;        // 0=all off, 1/2=pump (manual mode)
bool wellOk        = false;    // true when water is available in the well
volatile byte tankStep = 1;    // 1..2, alternates pumps on tank events

struct PumpTime {
  byte hours;
  byte minutes;
  byte seconds;
};
PumpTime pump[2] = { {0,0,0}, {0,0,0} };

// rotation state (timed control)
byte activePump = 255;          // 255=none, 0/1=pump
RtcDateTime pumpStart;

// ---------------- Protections ----------------
byte dryMin = 0;                 // dry-run grace, minutes (0=disabled)
byte voltLow = 10;               // low voltage threshold in volts (0=disabled)
uint16_t maintHours = 1000;      // maintenance interval (0=disabled)
bool dryFault   = false;
bool voltFault  = false;
bool pumpFault  = false;
bool maintDue   = false;
unsigned long drySince   = 0;    // when well turned dry
unsigned long vtBadSince = 0;    // voltage below threshold since
unsigned long vtGoodSince = 0;   // voltage recovered since
unsigned long runSeconds[2] = { 0, 0 };

// ---------------- Shield buttons ----------------
enum Button { B_NONE, B_RIGHT, B_UP, B_DOWN, B_LEFT, B_SELECT };

Button readButtons() {
  int v = analogRead(A0);
  if (v < 80)  return B_RIGHT;
  if (v < 220) return B_UP;
  if (v < 420) return B_DOWN;
  if (v < 620) return B_LEFT;
  if (v < 850) return B_SELECT;
  return B_NONE;
}

// ---------------- Menu ----------------
const byte MENU_COUNT = 10;
const byte ITEM_MODE=0, ITEM_CONTROL=1, ITEM_DRY=2, ITEM_VOLT=3,
           ITEM_PUMPTIME=4, ITEM_RUNTIME=5, ITEM_EVENTS=6,
           ITEM_FAULTS=7, ITEM_CLOCK=8, ITEM_BACK=9;
const char menuItems[MENU_COUNT][9] PROGMEM =
  { "Mode", "Control", "DryRun", "Voltage", "PumpTime",
    "RunTime", "Events", "Faults", "Clock", "Back" };

// edit states
const byte EDIT_NONE=0;
const byte EDIT_MODE=1, EDIT_CONTROL=2, EDIT_DRY=3, EDIT_VOLT=4,
           EDIT_PICK_PUMP=5, EDIT_PUMP_H=6, EDIT_PUMP_M=7, EDIT_PUMP_S=8,
           EDIT_CLOCK_H=9, EDIT_CLOCK_M=10, EDIT_CLOCK_S=11,
           EDIT_CLOCK_D=12, EDIT_CLOCK_MO=13,
           EDIT_RUNTIME=14, EDIT_EVENTS=15, EDIT_FAULTS=16;

bool inMenu   = false;
byte menuPos  = 0;
byte editState= EDIT_NONE;
byte editPump = 0;
byte evtView  = 0;
byte clk[5];   // H, M, S, day, month  (used while editing the clock)

// ============================================================
//  Forward declarations
// ============================================================
void line2Pad(byte from);
const char* modeShort();
const char* modeLong();
void allPumpsOff();
void setSinglePump(byte idx);
void loadPumpTimes();
void savePumpTime(byte m);
void addEvent(byte type);
void loadEvents();
void clearFaults();
void finishEdit();
void enterSelected();
void applyClock();
void changeVal(Button b, byte &v, byte maxVal);
void handleEdit(Button b);
void navigateMenu(Button b);
void handleButtons();
void applyManualPump();
void pumpBySensors();
bool pumpHasTime(byte i);
byte nextPumpWithTime(byte from);
void pumpByTimeRotation();
void handleDryProtection();
void handleVoltageProtection();
void interlockCheck();
void beepAlarm();
void print2(byte v);
void printMenuItem(byte idx);
byte menuItemLen(byte idx);
void drawStatus();
void drawPumpTimeValue();
void drawClockValue();
void drawItemValue(byte item);
void setBlink(byte col);
void drawEditValue();
void drawMenu();
void updateDisplay();
void processCommand(const char* cmd);
const char* skipToComma(const char* p);
void handleSerial();
void printEvents();
void printStatus();
void showHelp();
void sonidoTecla();
void sonidoGuardado();
void tankISR();

// ============================================================
//  Small helpers
// ============================================================
void line2Pad(byte from) {
  for (byte i = from; i < 16; i++) lcd.print(' ');
}

const char* modeShort() {
  if (mode == 1) return "AUTO";
  if (mode == 2) return "MAN";
  return "OFF";
}

const char* modeLong() {
  if (mode == 1) return "AUTO";
  if (mode == 2) return "MANUAL";
  return "OFF";
}

void allPumpsOff() {
  for (byte i = 0; i < 2; i++) digitalWrite(pumpPins[i], PUMP_OFF);
}

void setSinglePump(byte idx) {      // idx 255 => all off
  for (byte i = 0; i < 2; i++) digitalWrite(pumpPins[i], (i == idx) ? PUMP_ON : PUMP_OFF);
}

void copyEvtName(byte type, char* out) {   // out must hold >= 7 bytes
  byte idx = type - 1;
  if (idx >= 10) { *out = 0; return; }
  for (byte c = 0; c < 6; c++) {
    char ch = (char)pgm_read_byte(&evtNames[idx][c]);
    if (ch == 0) break;
    *out++ = ch;
  }
  *out = 0;
}

// ============================================================
//  EEPROM
// ============================================================
void loadPumpTimes() {
  for (byte m = 0; m < 2; m++) {
    EEPROM.get(addrPumpTime[m], pump[m]);
    if (pump[m].hours == 255) pump[m] = { 0, 0, 0 };  // fresh EEPROM
  }
}

void savePumpTime(byte m) {
  EEPROM.put(addrPumpTime[m], pump[m]);
}

void saveRuntimes() {
  EEPROM.put(addrRunP1, runSeconds[0]);
  EEPROM.put(addrRunP2, runSeconds[1]);
}

void loadEvents() {
  byte idx = EEPROM.read(addrLogIdx);
  byte full = EEPROM.read(addrLogFull);
  if (idx >= EVT_MAX) {
    evtIdx = 0; evtCount = 0;
    EEPROM.update(addrLogIdx, 0);
    EEPROM.update(addrLogFull, 0);
    return;
  }
  evtIdx = idx;
  evtCount = (full == 1) ? EVT_MAX : idx;
  for (byte i = 0; i < EVT_MAX; i++) EEPROM.get(addrLogBase + i * sizeof(LogEvent), evts[i]);
}

void addEvent(byte type) {
  RtcDateTime n = rtc.GetDateTime();
  LogEvent e;
  e.type = type;
  e.hh = n.Hour(); e.mm = n.Minute(); e.ss = n.Second();
  e.dd = n.Day();  e.mo = n.Month();

  byte pos = evtIdx;
  evts[pos] = e;
  evtIdx = (evtIdx + 1) % EVT_MAX;
  if (evtCount < EVT_MAX) {
    evtCount++;
    if (evtCount >= EVT_MAX) EEPROM.update(addrLogFull, 1);
  }
  EEPROM.put(addrLogBase + pos * sizeof(LogEvent), e);
  EEPROM.update(addrLogIdx, evtIdx);
}

void clearFaults() {
  if (pumpFault || dryFault || voltFault) addEvent(EVT_CLEAR);
  pumpFault = false; dryFault = false; voltFault = false;
  drySince = 0; vtBadSince = 0; vtGoodSince = 0;
}

// ============================================================
//  Button / menu logic
// ============================================================
void changeVal(Button b, byte &v, byte maxVal) {
  if (b == B_RIGHT) v = (v >= maxVal) ? 0 : (v + 1);
  else if (b == B_LEFT) v = (v == 0) ? maxVal : (v - 1);
  else if (b == B_UP) v = min(maxVal, (byte)(v + 10));
  else if (b == B_DOWN) v = (v >= 10) ? (v - 10) : 0;
}

void finishEdit() {
  editState = EDIT_NONE;
  sonidoGuardado();
}

void enterSelected() {
  switch (menuPos) {
    case ITEM_MODE:     editState = EDIT_MODE;      break;
    case ITEM_CONTROL:  editState = EDIT_CONTROL;   break;
    case ITEM_DRY:      editState = EDIT_DRY;       break;
    case ITEM_VOLT:     editState = EDIT_VOLT;      break;
    case ITEM_PUMPTIME: editState = EDIT_PICK_PUMP; editPump = 0; break;
    case ITEM_RUNTIME:  editState = EDIT_RUNTIME;   editPump = 0; break;
    case ITEM_EVENTS:   editState = EDIT_EVENTS;    evtView = 0;  break;
    case ITEM_FAULTS:   editState = EDIT_FAULTS;    break;
    case ITEM_CLOCK:
      {
        RtcDateTime n = rtc.GetDateTime();
        clk[0] = n.Hour();  clk[1] = n.Minute(); clk[2] = n.Second();
        clk[3] = n.Day();   clk[4] = n.Month();
        editState = EDIT_CLOCK_H;
      }
      break;
    case ITEM_BACK: inMenu = false; break;
  }
}

void handleEdit(Button b) {
  switch (editState) {
    case EDIT_MODE:
      if (b == B_RIGHT) mode = (mode + 1) % 3;
      else if (b == B_LEFT) mode = (mode + 2) % 3;
      else if (b == B_SELECT) { EEPROM.update(addrMode, mode); finishEdit(); }
      break;
    case EDIT_CONTROL:
      if (b == B_RIGHT || b == B_LEFT) controlByTime = !controlByTime;
      else if (b == B_SELECT) { EEPROM.update(addrControl, controlByTime); finishEdit(); }
      break;
    case EDIT_DRY:
      changeVal(b, dryMin, 30);
      if (b == B_SELECT) { EEPROM.update(addrDryMin, dryMin); finishEdit(); }
      break;
    case EDIT_VOLT:
      changeVal(b, voltLow, 13);
      if (b == B_SELECT) { EEPROM.update(addrVoltLow, voltLow); finishEdit(); }
      break;
    case EDIT_PICK_PUMP:
      if (b == B_RIGHT || b == B_LEFT) editPump = 1 - editPump;
      else if (b == B_SELECT) editState = EDIT_PUMP_H;
      break;
    case EDIT_PUMP_H:
      changeVal(b, pump[editPump].hours, 23);
      if (b == B_SELECT) editState = EDIT_PUMP_M;
      break;
    case EDIT_PUMP_M:
      changeVal(b, pump[editPump].minutes, 59);
      if (b == B_SELECT) editState = EDIT_PUMP_S;
      break;
    case EDIT_PUMP_S:
      changeVal(b, pump[editPump].seconds, 59);
      if (b == B_SELECT) { savePumpTime(editPump); finishEdit(); }
      break;
    case EDIT_RUNTIME:
      if (b == B_RIGHT || b == B_LEFT) editPump = 1 - editPump;
      else if (b == B_SELECT) {
        runSeconds[editPump] = 0;
        saveRuntimes();
        Serial.print(F("Pump ")); Serial.print(editPump + 1);
        Serial.println(F(" runtime reset"));
        finishEdit();
      }
      break;
    case EDIT_EVENTS:
      if (b == B_RIGHT)   { if (evtView > 0) evtView--; }
      else if (b == B_LEFT) { if (evtCount > 0 && evtView < evtCount - 1) evtView++; }
      else if (b == B_SELECT) finishEdit();
      break;
    case EDIT_FAULTS:
      if (b == B_SELECT) { clearFaults(); finishEdit(); }
      else finishEdit();
      break;
    case EDIT_CLOCK_H:
      changeVal(b, clk[0], 23);
      if (b == B_SELECT) editState = EDIT_CLOCK_M;
      break;
    case EDIT_CLOCK_M:
      changeVal(b, clk[1], 59);
      if (b == B_SELECT) editState = EDIT_CLOCK_S;
      break;
    case EDIT_CLOCK_S:
      changeVal(b, clk[2], 59);
      if (b == B_SELECT) editState = EDIT_CLOCK_D;
      break;
    case EDIT_CLOCK_D:
      changeVal(b, clk[3], 31);
      if (b == B_SELECT) editState = EDIT_CLOCK_MO;
      break;
    case EDIT_CLOCK_MO:
      changeVal(b, clk[4], 12);
      if (b == B_SELECT) { applyClock(); finishEdit(); }
      break;
  }
}

void applyClock() {
  RtcDateTime now = rtc.GetDateTime();
  rtc.SetDateTime(RtcDateTime(now.Year(), clk[4], clk[3], clk[0], clk[1], clk[2]));
  Serial.println(F("Date and time updated"));
}

void navigateMenu(Button b) {
  if (b == B_UP) menuPos = (menuPos + MENU_COUNT - 1) % MENU_COUNT;
  else if (b == B_DOWN) menuPos = (menuPos + 1) % MENU_COUNT;
  else if (b == B_SELECT) enterSelected();
}

void handleButtons() {
  Button b = readButtons();
  if (b == B_NONE) return;

  sonidoTecla();

  if (!inMenu) {
    if (b == B_RIGHT) manualPump = (manualPump + 1) % 3;
    else if (b == B_LEFT) manualPump = (manualPump + 2) % 3;
    else if (b == B_SELECT) { inMenu = true; menuPos = 0; }
    updateDisplay();
    delay(120);
    return;
  }

  if (editState != EDIT_NONE) handleEdit(b);
  else navigateMenu(b);

  updateDisplay();
  delay(120);
}

// ============================================================
//  Pump control
// ============================================================
void applyManualPump() {
  if (manualPump == 0) allPumpsOff();
  else setSinglePump(manualPump - 1);
}

void pumpBySensors() {
  if (digitalRead(sensor_tank) == LOW) setSinglePump(tankStep - 1);  // tank empty
  else allPumpsOff();                                                 // tank full
}

bool pumpHasTime(byte i) {
  return (pump[i].hours + pump[i].minutes + pump[i].seconds) > 0;
}

byte nextPumpWithTime(byte from) {
  for (byte k = 0; k < 2; k++) {
    byte i = (from + k) % 2;
    if (pumpHasTime(i)) return i;
  }
  return 255;   // none configured
}

void pumpByTimeRotation() {
  if (activePump == 255) {
    byte next = nextPumpWithTime(0);
    if (next == 255) { allPumpsOff(); return; }
    activePump = next;
    setSinglePump(activePump);
    pumpStart = rtc.GetDateTime();
    return;
  }

  RtcDateTime now = rtc.GetDateTime();
  unsigned long elapsed = (unsigned long)(now.TotalSeconds64() - pumpStart.TotalSeconds64());
  unsigned long target = pump[activePump].hours * 3600UL
                       + pump[activePump].minutes * 60UL
                       + pump[activePump].seconds;

  if (elapsed >= target) {
    byte next = nextPumpWithTime((activePump + 1) % 2);
    if (next == 255) { activePump = 255; allPumpsOff(); return; }
    activePump = next;
    setSinglePump(activePump);
    pumpStart = now;
  }
}

// ============================================================
//  Protections
// ============================================================
void handleDryProtection() {
  if (!wellOk) {
    if (drySince == 0) drySince = millis();
    unsigned long dryMs = millis() - drySince;
    if (dryMin > 0 && !dryFault && (dryMs >= (unsigned long)dryMin * 60000UL)) {
      dryFault = true;
      addEvent(EVT_DRY);
      beepAlarm();
    }
  } else {
    drySince = 0;
    if (dryFault) {
      dryFault = false;
      addEvent(EVT_DRY_OK);
    }
  }
}

void handleVoltageProtection() {
  static unsigned long lastRead = 0;
  static byte v100 = 156;
  if (millis() - lastRead > 250) {
    lastRead = millis();
    v100 = (byte)(((unsigned long)analogRead(voltagePin) * 156UL) / 1023UL);
  }
  if (voltLow == 0) { vtBadSince = 0; vtGoodSince = 0; return; }

  byte th = voltLow * 10;
  if (v100 < th) {
    vtGoodSince = 0;
    if (vtBadSince == 0) vtBadSince = millis();
    else if (!voltFault && (millis() - vtBadSince > 2000)) {
      voltFault = true;
      addEvent(EVT_VOLT);
      beepAlarm();
    }
  } else {
    vtBadSince = 0;
    if (voltFault) {
      if (vtGoodSince == 0) vtGoodSince = millis();
      else if (millis() - vtGoodSince > 5000) {
        voltFault = false;
        addEvent(EVT_VOLT_OK);
      }
    }
  }
}

void interlockCheck() {
  if (digitalRead(pumpPins[0]) == PUMP_ON && digitalRead(pumpPins[1]) == PUMP_ON) {
    allPumpsOff();
    activePump = 255;
    if (!pumpFault) {
      pumpFault = true;
      addEvent(EVT_BOTH);
      beepAlarm();
    }
  }
}

// ============================================================
//  Display
// ============================================================
void print2(byte v) {
  if (v < 10) lcd.print('0');
  lcd.print(v);
}

void printMenuItem(byte idx) {
  for (byte c = 0; c < 9; c++) {
    char ch = (char)pgm_read_byte(&menuItems[idx][c]);
    if (ch == 0) break;
    lcd.print(ch);
  }
}

byte menuItemLen(byte idx) {
  byte l = 0;
  for (byte c = 0; c < 9; c++) {
    if (pgm_read_byte(&menuItems[idx][c]) == 0) break;
    l++;
  }
  return l;
}

void drawStatus() {
  lcd.setCursor(0, 0);
  lcd.print(modeShort());
  lcd.print(" ");
  for (byte i = 0; i < 2; i++) {
    lcd.print(digitalRead(pumpPins[i]) == PUMP_ON ? '1' : '0');
  }
  line2Pad(7);

  lcd.setCursor(0, 1);
  if (pumpFault) { lcd.print(F("BOTH-FAULT!")); line2Pad(11); }
  else if (voltFault) { lcd.print(F("VOLT LOW!")); line2Pad(10); }
  else if (dryFault) { lcd.print(F("DRY FAULT!")); line2Pad(10); }
  else if (maintDue) { lcd.print(F("MAINT DUE!")); line2Pad(10); }
  else if (!wellOk) { lcd.print(F("WELL:EMPTY")); line2Pad(10); }
  else {
    RtcDateTime n = rtc.GetDateTime();
    print2(n.Hour()); lcd.print(':');
    print2(n.Minute()); lcd.print(':');
    print2(n.Second()); lcd.print(' ');
    print2(n.Day()); lcd.print('/');
    print2(n.Month());
    line2Pad(14);
  }
}

void drawPumpTimeValue() {
  char buf[17];
  snprintf_P(buf, sizeof(buf), PSTR("P%d %02d:%02d:%02d"),
             editPump + 1, pump[editPump].hours,
             pump[editPump].minutes, pump[editPump].seconds);
  lcd.print(buf);
  line2Pad(strlen(buf));
}

void drawClockValue() {
  char buf[17];
  snprintf_P(buf, sizeof(buf), PSTR("%02d:%02d:%02d %02d/%02d"),
             clk[0], clk[1], clk[2], clk[3], clk[4]);
  lcd.print(buf);
  line2Pad(strlen(buf));
}

void drawRuntimeValue(bool editing) {
  char buf[17];
  unsigned long h = runSeconds[editPump] / 3600UL;
  unsigned long m = (runSeconds[editPump] % 3600UL) / 60UL;
  if (editing) {
    snprintf_P(buf, sizeof(buf), PSTR("R%d %04luh %02lum"), editPump + 1, h, m);
  } else {
    unsigned long h1 = runSeconds[0] / 3600UL;
    unsigned long h2 = runSeconds[1] / 3600UL;
    snprintf_P(buf, sizeof(buf), PSTR("R1 %luh R2 %luh"), h1, h2);
  }
  lcd.print(buf);
  line2Pad(strlen(buf));
}

void drawItemValue(byte item) {
  switch (item) {
    case ITEM_MODE:
      lcd.print(F("Mode: ")); lcd.print(modeLong()); line2Pad(10); break;
    case ITEM_CONTROL:
      lcd.print(controlByTime ? F("Ctrl: TIME") : F("Ctrl: SENSOR")); line2Pad(11); break;
    case ITEM_DRY:
      if (dryMin == 0) lcd.print(F("DryM: OFF"));
      else { lcd.print(F("DryM: ")); lcd.print(dryMin); lcd.print(F("m")); }
      line2Pad(9);
      break;
    case ITEM_VOLT:
      if (voltLow == 0) lcd.print(F("LowV: OFF"));
      else { lcd.print(F("LowV: ")); lcd.print(voltLow); lcd.print(F("V")); }
      line2Pad(9);
      break;
    case ITEM_PUMPTIME:
      editPump = 0;
      drawPumpTimeValue();
      break;
    case ITEM_RUNTIME:
      drawRuntimeValue(false);
      break;
    case ITEM_EVENTS:
      lcd.print(F("Last events   ")); break;
    case ITEM_FAULTS:
      lcd.print(F("D:")); lcd.print(dryFault ? '1' : '0');
      lcd.print(F(" V:")); lcd.print(voltFault ? '1' : '0');
      lcd.print(F(" B:")); lcd.print(pumpFault ? '1' : '0');
      lcd.print(F(" M:")); lcd.print(maintDue ? '1' : '0');
      line2Pad(15);
      break;
    case ITEM_CLOCK:
      {
        RtcDateTime n = rtc.GetDateTime();
        lcd.print(F("Now: "));
        print2(n.Hour()); lcd.print(':'); print2(n.Minute()); lcd.print(':');
        print2(n.Second());
        line2Pad(13);
      }
      break;
    case ITEM_BACK:
      lcd.print(F("Exit to status ")); break;
  }
}

void setBlink(byte col) {
  lcd.setCursor(col, 1);
  lcd.blink();
}

void drawEditValue() {
  lcd.noBlink();
  switch (editState) {
    case EDIT_MODE:
      lcd.print(F("> ")); lcd.print(modeLong()); line2Pad(7); break;
    case EDIT_CONTROL:
      lcd.print(controlByTime ? F("> TIME") : F("> SENSOR")); line2Pad(8); break;
    case EDIT_DRY:
      if (dryMin == 0) lcd.print(F("> OFF"));
      else { lcd.print(F("> ")); lcd.print(dryMin); lcd.print(F("m")); }
      line2Pad(5);
      break;
    case EDIT_VOLT:
      if (voltLow == 0) lcd.print(F("> OFF"));
      else { lcd.print(F("> ")); lcd.print(voltLow); lcd.print(F("V")); }
      line2Pad(5);
      break;
    case EDIT_PICK_PUMP:
      lcd.print(F("Pump: ")); lcd.print(editPump + 1); line2Pad(7); break;
    case EDIT_PUMP_H:
      drawPumpTimeValue(); setBlink(3); break;
    case EDIT_PUMP_M:
      drawPumpTimeValue(); setBlink(6); break;
    case EDIT_PUMP_S:
      drawPumpTimeValue(); setBlink(9); break;
    case EDIT_RUNTIME:
      drawRuntimeValue(true);
      break;
    case EDIT_EVENTS:
      {
        if (evtCount == 0) { lcd.print(F("(no events)  ")); break; }
        byte pos = (byte)((evtIdx - 1 - evtView + EVT_MAX) % EVT_MAX);
        char nm[7];
        copyEvtName(evts[pos].type, nm);
        char buf[17];
        snprintf(buf, sizeof(buf), "%02d:%02d:%02d %s",
                 evts[pos].hh, evts[pos].mm, evts[pos].ss, nm);
        lcd.print(buf); line2Pad(strlen(buf));
      }
      break;
    case EDIT_FAULTS:
      lcd.print(F("D:")); lcd.print(dryFault ? '1' : '0');
      lcd.print(F(" V:")); lcd.print(voltFault ? '1' : '0');
      lcd.print(F(" B:")); lcd.print(pumpFault ? '1' : '0');
      line2Pad(13);
      break;
    case EDIT_CLOCK_H:
      drawClockValue(); setBlink(0); break;
    case EDIT_CLOCK_M:
      drawClockValue(); setBlink(3); break;
    case EDIT_CLOCK_S:
      drawClockValue(); setBlink(6); break;
    case EDIT_CLOCK_D:
      drawClockValue(); setBlink(9); break;
    case EDIT_CLOCK_MO:
      drawClockValue(); setBlink(12); break;
  }
}

void drawMenu() {
  lcd.noBlink();
  lcd.setCursor(0, 0);
  lcd.print(menuPos == 0 ? '>' : ' ');
  printMenuItem(menuPos);
  line2Pad(menuItemLen(menuPos) + 1);

  lcd.setCursor(0, 1);
  if (editState == EDIT_NONE) drawItemValue(menuPos);
  else drawEditValue();
}

void updateDisplay() {
  if (inMenu) drawMenu();
  else drawStatus();
}

// ============================================================
//  Serial commands
// ============================================================
const char* skipToComma(const char* p) {
  while (*p && *p != ',') p++;
  return (*p == ',') ? p + 1 : p;
}

void processCommand(const char* cmd) {
  if (strcmp(cmd, "status") == 0)               printStatus();
  else if (strcmp(cmd, "mode0") == 0)           { mode = 0; EEPROM.update(addrMode, mode); Serial.println(F("Mode: OFF")); sonidoGuardado(); }
  else if (strcmp(cmd, "mode1") == 0)           { mode = 1; EEPROM.update(addrMode, mode); Serial.println(F("Mode: AUTO")); sonidoGuardado(); }
  else if (strcmp(cmd, "mode2") == 0)           { mode = 2; EEPROM.update(addrMode, mode); Serial.println(F("Mode: MANUAL")); sonidoGuardado(); }
  else if (strcmp(cmd, "pump1") == 0)           { manualPump = 1; Serial.println(F("Pump 1 selected (manual only)")); }
  else if (strcmp(cmd, "pump2") == 0)           { manualPump = 2; Serial.println(F("Pump 2 selected (manual only)")); }
  else if (strcmp(cmd, "pumpoff") == 0)         { manualPump = 0; Serial.println(F("Pumps off")); }
  else if (strcmp(cmd, "control_sensor") == 0)  { controlByTime = false; EEPROM.update(addrControl, false); Serial.println(F("Control: sensors")); sonidoGuardado(); }
  else if (strcmp(cmd, "control_time") == 0)    { controlByTime = true;  EEPROM.update(addrControl, true);  Serial.println(F("Control: time")); sonidoGuardado(); }
  else if (strncmp(cmd, "drymin ", 7) == 0) {
    byte n = (byte)atoi(cmd + 7);
    if (n <= 30) { dryMin = n; EEPROM.update(addrDryMin, dryMin); Serial.print(F("Dry-run grace: ")); Serial.println(dryMin); sonidoGuardado(); }
    else Serial.println(F("Error: 0..30 minutes"));
  }
  else if (strncmp(cmd, "voltlow ", 8) == 0) {
    byte v = (byte)atoi(cmd + 8);
    if (v <= 13) { voltLow = v; EEPROM.update(addrVoltLow, voltLow); Serial.print(F("Low voltage: ")); Serial.println(voltLow); sonidoGuardado(); }
    else Serial.println(F("Error: 0..13 volts"));
  }
  else if (strncmp(cmd, "maint ", 6) == 0) {
    unsigned int h = (unsigned int)atoi(cmd + 6);
    if (h <= 3000) { maintHours = h; EEPROM.put(addrMaintHours, maintHours); Serial.print(F("Maintenance interval: ")); Serial.println(maintHours); sonidoGuardado(); }
    else Serial.println(F("Error: 0..3000 hours"));
  }
  else if (strcmp(cmd, "runtime") == 0) {
    Serial.print(F("Pump1 runtime: ")); Serial.print(runSeconds[0] / 3600UL); Serial.println(F(" h"));
    Serial.print(F("Pump2 runtime: ")); Serial.print(runSeconds[1] / 3600UL); Serial.println(F(" h"));
  }
  else if (strncmp(cmd, "runtime_pump ", 13) == 0) {
    byte p = (byte)atoi(cmd + 13);
    if (p == 1 || p == 2) { runSeconds[p-1] = 0; saveRuntimes(); Serial.print(F("Pump runtime reset: ")); Serial.println(p); }
    else Serial.println(F("Error: 1 or 2"));
  }
  else if (strcmp(cmd, "events") == 0)          printEvents();
  else if (strcmp(cmd, "faults") == 0) {
    Serial.print(F("Dry:")); Serial.print(dryFault);
    Serial.print(F(" Volt:")); Serial.print(voltFault);
    Serial.print(F(" Both:")); Serial.print(pumpFault);
    Serial.print(F(" Maint:")); Serial.println(maintDue);
  }
  else if (strcmp(cmd, "fault_clear") == 0)     { clearFaults(); Serial.println(F("Faults cleared")); sonidoGuardado(); }
  else if (strncmp(cmd, "pump_time,", 10) == 0) {
    const char* p = cmd + 10;
    int m = atoi(p) - 1;
    p = skipToComma(p);
    int h = atoi(p); p = skipToComma(p);
    int mi = atoi(p); p = skipToComma(p);
    int s = atoi(p);
    if (m == 0 || m == 1) {
      pump[m].hours = h; pump[m].minutes = mi; pump[m].seconds = s;
      savePumpTime(m);
      Serial.print(F("Pump ")); Serial.print(m + 1);
      Serial.print(F(" configured: "));
      Serial.print(h); Serial.print(F(":")); Serial.print(mi); Serial.print(F(":")); Serial.println(s);
      sonidoGuardado();
    } else Serial.println(F("Error: pump must be 1 or 2"));
  }
  else if (strncmp(cmd, "set_time,", 9) == 0) {
    const char* p = cmd + 9;
    int h = atoi(p); p = skipToComma(p);
    int mi = atoi(p); p = skipToComma(p);
    int s = atoi(p); p = skipToComma(p);
    int d = atoi(p); p = skipToComma(p);
    int mo = atoi(p); p = skipToComma(p);
    int y = atoi(p);
    if (y > 0) {
      rtc.SetDateTime(RtcDateTime(2000 + y, mo, d, h, mi, s));
      Serial.println(F("Date and time updated"));
      sonidoGuardado();
    } else Serial.println(F("Error: format must be set_time,H,M,S,D,Mo,Y"));
  }
  else if (strcmp(cmd, "help") == 0)            showHelp();
  else Serial.println(F("Unknown command. Type 'help' for the list."));

  updateDisplay();
}

void handleSerial() {
  static char line[32];
  static byte len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      line[len] = 0;
      len = 0;
      processCommand(line);
    } else if (c != '\r' && len < 31) {
      line[len++] = c;
    }
  }
}

void printEvents() {
  Serial.println(F("=== EVENT LOG (newest first) ==="));
  if (evtCount == 0) { Serial.println(F("(empty)")); return; }
  for (byte v = 0; v < evtCount; v++) {
    byte pos = (byte)((evtIdx - 1 - v + EVT_MAX) % EVT_MAX);
    char nm[7];
    copyEvtName(evts[pos].type, nm);
    Serial.print(evts[pos].dd); Serial.print('/'); Serial.print(evts[pos].mo); Serial.print(' ');
    if (evts[pos].hh < 10) Serial.print('0');
    Serial.print(evts[pos].hh); Serial.print(':');
    if (evts[pos].mm < 10) Serial.print('0');
    Serial.print(evts[pos].mm); Serial.print(':');
    if (evts[pos].ss < 10) Serial.print('0');
    Serial.print(evts[pos].ss); Serial.print(' ');
    Serial.println(nm);
  }
  Serial.println(F("================================"));
}

void printStatus() {
  RtcDateTime n = rtc.GetDateTime();
  Serial.println(F("==== CURRENT STATUS ===="));
  Serial.print(F("Date:   ")); Serial.print(n.Day()); Serial.print(F("/"));
  Serial.print(n.Month()); Serial.print(F("/")); Serial.println(n.Year());
  Serial.print(F("Time:   "));
  if (n.Hour() < 10) Serial.print(F("0"));
  Serial.print(n.Hour()); Serial.print(F(":"));
  if (n.Minute() < 10) Serial.print(F("0"));
  Serial.print(n.Minute()); Serial.print(F(":"));
  if (n.Second() < 10) Serial.print(F("0"));
  Serial.println(n.Second());
  Serial.print(F("Mode:   "));
  if (mode == 0) Serial.println(F("OFF"));
  else if (mode == 1) Serial.println(F("AUTO"));
  else Serial.println(F("MANUAL"));
  Serial.print(F("Control:")); Serial.println(controlByTime ? F(" by time") : F(" by sensors"));
  Serial.print(F("Well:   ")); Serial.println(wellOk ? F("water available") : F("empty"));
  Serial.print(F("Tank:   ")); Serial.println(digitalRead(sensor_tank) == LOW ? F("empty") : F("full"));
  Serial.print(F("Voltage:")); Serial.println(((unsigned long)analogRead(voltagePin) * 156UL) / 1023UL / 10);
  Serial.print(F("DryMsg: ")); Serial.println(dryFault ? F("FAULT") : F("ok"));
  Serial.print(F("VoltFa: ")); Serial.println(voltFault ? F("FAULT") : F("ok"));
  Serial.print(F("BothFa: ")); Serial.println(pumpFault ? F("FAULT") : F("ok"));
  Serial.print(F("Maint:  ")); Serial.println(maintDue ? F("DUE") : F("ok"));
  Serial.print(F("DryMin: ")); Serial.println(dryMin);
  Serial.print(F("LowVolt:")); Serial.println(voltLow);
  Serial.print(F("MaintH: ")); Serial.println(maintHours);
  Serial.print(F("Run P1: ")); Serial.print(runSeconds[0] / 3600UL); Serial.println(F(" h"));
  Serial.print(F("Run P2: ")); Serial.print(runSeconds[1] / 3600UL); Serial.println(F(" h"));
  Serial.println(F("Pump times:"));
  for (byte i = 0; i < 2; i++) {
    Serial.print(F("  Pump ")); Serial.print(i + 1); Serial.print(F(": "));
    Serial.print(pump[i].hours); Serial.print(F(":"));
    if (pump[i].minutes < 10) Serial.print(F("0"));
    Serial.print(pump[i].minutes); Serial.print(F(":"));
    if (pump[i].seconds < 10) Serial.print(F("0"));
    Serial.println(pump[i].seconds);
  }
  Serial.println(F("========================"));
}

void showHelp() {
  Serial.println(F("==== AVAILABLE COMMANDS ===="));
  Serial.println(F("status                      - current status"));
  Serial.println(F("mode0 / mode1 / mode2       - OFF / AUTO / MANUAL"));
  Serial.println(F("pump1 / pump2 / pumpoff     - manual pump select"));
  Serial.println(F("control_sensor|control_time - switch control type"));
  Serial.println(F("drymin N                    - dry-run grace minutes (0=off)"));
  Serial.println(F("voltlow V                   - low voltage threshold (0=off)"));
  Serial.println(F("maint H                     - maintenance interval hours"));
  Serial.println(F("runtime                     - pump runtime hours"));
  Serial.println(F("runtime_pump P              - reset runtime of pump P"));
  Serial.println(F("events                      - event log"));
  Serial.println(F("faults | fault_clear        - show / clear faults"));
  Serial.println(F("pump_time,M,h,m,s           - set run time of pump M"));
  Serial.println(F("set_time,H,M,S,D,Mo,Y       - set date and time"));
  Serial.println(F("help                        - this list"));
  Serial.println(F("================================="));
}

// ============================================================
//  Buzzer
// ============================================================
void sonidoTecla() {
  tone(buzzer, toneKey, durShort);
  delay(durShort);
}
void sonidoGuardado() {
  tone(buzzer, toneSaved, durLong);
  delay(durLong);
}
void beepAlarm() {
  for (byte i = 0; i < 3; i++) {
    tone(buzzer, 800, 120);
    delay(150);
  }
}

// ============================================================
//  Tank sensor interrupt
// ============================================================
void tankISR() {
  tankStep++;
  if (tankStep > 2) tankStep = 1;
}

// ============================================================
//  setup / loop
// ============================================================
void setup() {
  Serial.begin(9600);
  Serial.println(F("Water pump control system started"));
  Serial.println(F("Water pump v2 - protections, runtime, event log"));

  rtc.Begin();
  // uncomment once to set RTC from PC build time (replace the void battery/new module):
  // rtc.SetDateTime(RtcDateTime(__DATE__, __TIME__));
  if (!rtc.IsDateTimeValid()) {
    Serial.println(F("RTC has invalid time, setting default 2026-01-01 12:00"));
    rtc.SetDateTime(RtcDateTime(2026, 1, 1, 12, 0, 0));
  }
  {
    RtcDateTime boot = rtc.GetDateTime();
    Serial.print(F("RTC valid="));
    Serial.println(rtc.IsDateTimeValid() ? F("yes") : F("no"));
    Serial.print(F("RTC now="));
    Serial.print(boot.Year()); Serial.print(F("/"));
    Serial.print(boot.Month()); Serial.print(F("/"));
    Serial.print(boot.Day()); Serial.print(F(" "));
    Serial.print(boot.Hour()); Serial.print(F(":"));
    Serial.print(boot.Minute()); Serial.print(F(":"));
    Serial.println(boot.Second());
  }

  mode          = EEPROM.read(addrMode);
  if (mode > 2) mode = 0;
  byte ctrl = EEPROM.read(addrControl);
  controlByTime = (ctrl == 1);

  dryMin = EEPROM.read(addrDryMin);
  if (dryMin > 30) dryMin = 0;
  voltLow = EEPROM.read(addrVoltLow);
  if (voltLow > 13) voltLow = 0;
  EEPROM.get(addrMaintHours, maintHours);
  if (maintHours == 0xFFFF || maintHours > 3000) maintHours = 1000;

  EEPROM.get(addrRunP1, runSeconds[0]);
  EEPROM.get(addrRunP2, runSeconds[1]);
  loadPumpTimes();
  loadEvents();

  pinMode(sensor_well, INPUT);
  pinMode(sensor_tank, INPUT);
  digitalWrite(pump1, PUMP_OFF);   // latch OFF before enabling output (active-LOW)
  digitalWrite(pump2, PUMP_OFF);
  pinMode(pump1, OUTPUT);
  pinMode(pump2, OUTPUT);
  pinMode(voltagePin, INPUT);
  pinMode(ledAuto, OUTPUT);
  pinMode(buzzer, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(sensor_tank), tankISR, FALLING);

  lcd.begin(16, 2);
  addEvent(EVT_BOOT);
  updateDisplay();
  printStatus();
}

void loop() {
  wellOk = (digitalRead(sensor_well) == LOW);

  digitalWrite(ledAuto,   (mode == 1) ? HIGH : LOW);

  handleDryProtection();
  handleVoltageProtection();

  bool blocked = dryFault || voltFault || pumpFault;
  if (blocked || (!wellOk && dryMin == 0)) {
    allPumpsOff();
    activePump = 255;
  } else if (mode == 1) {
    if (!controlByTime) pumpBySensors();
    else pumpByTimeRotation();
  } else if (mode == 2) {
    applyManualPump();
    activePump = 255;
  } else {
    allPumpsOff();
    activePump = 255;
  }

  interlockCheck();

  // runtime counter (1 second tick)
  static unsigned long secTick = 0;
  static unsigned long lastRunSave = 0;
  if (millis() - secTick >= 1000) {
    secTick = millis();
    for (byte i = 0; i < 2; i++) {
      if (digitalRead(pumpPins[i]) == PUMP_ON) runSeconds[i]++;
    }
    if (millis() - lastRunSave >= 60000) {
      lastRunSave = millis();
      saveRuntimes();
    }
  }

  // pump start/stop events
  static byte prevPump[2] = { 0, 0 };
  for (byte i = 0; i < 2; i++) {
    byte cur = (digitalRead(pumpPins[i]) == PUMP_ON) ? 1 : 0;
    if (cur != prevPump[i]) {
      prevPump[i] = cur;
      addEvent(cur ? EVT_START : EVT_STOP);
    }
  }

  // maintenance reminder
  static bool prevMaint = false;
  maintDue = false;
  if (maintHours > 0) {
    for (byte i = 0; i < 2; i++) {
      if ((runSeconds[i] / 3600UL) >= maintHours) maintDue = true;
    }
  }
  if (maintDue != prevMaint) {
    prevMaint = maintDue;
    if (maintDue) addEvent(EVT_MAINT);
  }

  handleButtons();
  handleSerial();

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 15000) {
    printStatus();
    lastPrint = millis();
  }

  updateDisplay();
}