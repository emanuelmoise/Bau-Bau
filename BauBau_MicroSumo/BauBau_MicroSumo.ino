/*
 * =====================================================================
 *  BauBau - Micro Sumo  (JSumo XS1 / XMotion Micro, ATmega32u4)
 *  Cod pentru RoboChallenge - categoria Micro Sumo
 * ---------------------------------------------------------------------
 *  Arduino IDE:  Tools -> Board -> "Arduino Leonardo", apoi Tools -> Port
 *
 *  Logica hardware (din codul si documentatia oficiala JSumo pentru XS1):
 *   - JS40F (adversar) : HIGH (1) cand vede adversarul
 *   - ML1   (linie)    : valoare analogica MICA (~0) pe linia alba, mare pe negru
 *   - Butoane          : LOW (0) cand sunt apasate (INPUT_PULLUP)
 *   - LED-uri          : HIGH = aprins
 *   - Motoare          : pin DIR + pin PWM, exact schema din codul oficial
 *
 *  BUTOANE (in asteptare):
 *   - apasare scurta STANGA  -> tactica "cauta spre STANGA"
 *   - apasare scurta DREAPTA -> tactica "cauta spre DREAPTA"
 *   - ambele butoane         -> tactica "DREPT inainte"
 *     * fara modul de start (USE_START_MODULE 0): apasarea porneste si
 *       numaratoarea de START_DELAY_MS (5 s), apoi lupta
 *     * cu modul de start   (USE_START_MODULE 1): lupta porneste de la
 *       telecomanda arbitrului; tine apasat 1,5 s ca sa pornesti fara telecomanda
 *   - orice buton in timpul luptei -> STOP
 *
 *  LED-uri in asteptare: LED1 = un senzor vede adversarul, LED2 = senzorul de
 *  linie vede ALB (verifica robotul pe dohyo inainte de meci!)
 *
 *  MODURI SPECIALE (tine butonul apasat cand pornesti robotul de la intrerupator):
 *   - STANGA  tinut -> mod TEST (telemetrie pe Serial Monitor 115200;
 *                      STANGA = test motoare, DREAPTA = calibrare linie)
 *   - DREAPTA tinut -> calibrare senzor de linie (se salveaza in EEPROM)
 *
 *  COMENZI SERIAL (optional, cu cablul USB si Serial Monitor la 115200):
 *   t = telemetrie on/off, m = test motoare, c = calibrare linie,
 *   d = prag linie implicit, 1/2/3 = tactica stanga/drept/dreapta,
 *   s = start imediat (tine robotul in aer!), x = stop, ? = ajutor
 * =====================================================================
 */

#include <Arduino.h>
#include <EEPROM.h>

// =====================================================================
//  1. CONFIGURARE  (tot ce ai nevoie sa reglezi este aici)
// =====================================================================

// --- 1.1 Modul de start/stop IR (standard startmodule.com) - OBLIGATORIU la RoboChallenge
//     Pinul "Start" al modulului: 0 la pornire, 1 cand arbitrul da START, 0 la STOP.
//     Pinul "Kill" (optional):     1 la pornire, 0 cand arbitrul da STOP.
#define USE_START_MODULE      0     // 1 = modulul este conectat (OBLIGATORIU la concurs); 0 = doar butoane
#define PIN_START_MODULE      2     // terminalul liber de pe placa la care ai legat pinul "Start"
#define USE_KILL_PIN          0     // 1 = ai legat si pinul "Kill" al modulului
#define PIN_KILL_MODULE       3     // terminalul liber la care ai legat pinul "Kill"

// --- 1.2 Pornire de la butoane
#define START_DELAY_MS     5000UL   // numaratoare dupa apasare (regula clasica de 5 s); 0 = imediat
#define LONG_PRESS_MS      1500UL   // cu modul de start: tine apasat atat ca sa pornesti fara telecomanda

// --- 1.3 Viteze (0..255)
#define SPEED_ATTACK          255   // adversar in fata: impinge cu tot
#define SPEED_ATTACK_ARC      150   // roata interioara cand adversarul e in fata + lateral (arc)
#define SPEED_TURN            190   // rotire pe loc spre adversarul vazut lateral
#define SPEED_SEARCH_SPIN     150   // rotire de cautare (prea rapid = senzorii nu apuca sa vada)
#define SPEED_SEARCH_FWD      160   // inaintare de cautare (moderat: nu ne aruncam in margine)
#define SPEED_EDGE_BACK       255   // retragere de la margine
#define SPEED_EDGE_TURN       200   // intoarcere dupa retragere

// --- 1.4 Timpi (ms)
#define EDGE_BACK_MS          150   // cat dam inapoi la margine (dohyo mic, 38,5 cm - nu exagera)
#define EDGE_TURN_MS          180   // cat ne rotim dupa retragere (tinta: ~120-150 de grade)
#define EDGE_TURN_MIN_MS      100   // in prima parte a rotirii ignoram senzorii (vad obiecte din afara dohyo-ului)
#define SEARCH_SPIN_MS        700   // rotire de cautare, apoi
#define SEARCH_FWD_MS         350   // ...o bucata inainte, si tot asa
#define INITIAL_SPIN_MS       500   // tactica stanga/dreapta: cat ne rotim la start daca nu vedem nimic
#define OPP_LOST_MS            60   // cat pastram ultima comanda cand senzorii "clipesc"
#define EDGE_PUSH_OVERRIDE_MS   0   // RISCANT: daca >0, cu adversarul in fata ignoram linia atatea ms. Lasa 0.

// --- 1.5 Senzor de linie (ML1). Valorile de aici se folosesc daca nu ai calibrat.
#define LINE_THRESHOLD_DEFAULT     45   // sub 45/1023 = alb (valoarea din codul oficial JSumo)
#define LINE_WHITE_IS_LOW_DEFAULT   1   // 1 = alb da valoare mica (ML1)
#define LINE_CONFIRM_SAMPLES        2   // citiri consecutive "alb" ca sa declaram margine (anti-zgarieturi)

// --- 1.6 Corectii hardware (schimba DOAR daca testul de motoare/senzori arata altceva)
#define OPP_ACTIVE_HIGH        1    // 1 = JS40F da HIGH cand vede (asa este la kit)
#define MOTOR_SWAP             0    // 1 = roata stanga/dreapta inversate in testul de motoare
#define MOTOR_LEFT_INVERT      0    // 1 = roata stanga se invarte invers
#define MOTOR_RIGHT_INVERT     0    // 1 = roata dreapta se invarte invers
#define PWM_HIGH_FREQ          0    // 1 = PWM ~31 kHz (fara fluierat); lasa 0 daca nu esti sigur de driver

// --- 1.7 Diverse
#define BTN_DEBOUNCE_MS        30
#define STOP_HOLD_MS           30   // buton tinut atat in lupta = STOP (filtru de zgomot electric)
#define MODULE_DEBOUNCE_MS     20   // semnalul modulului trebuie sa fie stabil atat
#define MODULE_ARM_MS          50   // modulul trebuie vazut in "0" atat inainte sa acceptam un start
#define SERIAL_BAUD        115200

// =====================================================================
//  2. PINI  (identici cu codul oficial JSumo XS1 - nu modifica)
// =====================================================================
#define PIN_OPP_RIGHT   A0
#define PIN_OPP_FRONT   A1
#define PIN_OPP_LEFT    A2
#define PIN_LINE        A3
#define PIN_BTN_RIGHT   8
#define PIN_BTN_LEFT    12
#define PIN_LED1        4
#define PIN_LED2        6
// In codul oficial JSumo valoarea pentru roata STANGA se scrie pe pinii 10/9
// (numiti acolo "Right_Motor") si cea pentru roata DREAPTA pe pinii 13/5
// ("Left_Motor"). Pastram exact acest comportament, pentru ca este cel testat
// pe kit. Daca testul de motoare arata altceva, foloseste MOTOR_SWAP / *_INVERT.
#define PIN_ML_DIR      10
#define PIN_ML_PWM      9
#define PIN_MR_DIR      13
#define PIN_MR_PWM      5

// =====================================================================
//  3. TIPURI SI STARE
// =====================================================================
enum Tactic     : uint8_t { TACTIC_LEFT, TACTIC_STRAIGHT, TACTIC_RIGHT };
enum MainState  : uint8_t { ST_IDLE, ST_COUNTDOWN, ST_FIGHT };
enum FightState : uint8_t { FS_SEARCH_SPIN, FS_SEARCH_FWD, FS_ATTACK, FS_EDGE_BACK, FS_EDGE_TURN };

#define DIR_LEFT   (-1)
#define DIR_RIGHT  ( 1)

struct Sensors {
  bool oppL, oppF, oppR;   // adversar stanga / fata / dreapta
  bool line;               // linie alba confirmata
  int  lineRaw;            // citirea bruta 0..1023
  bool btnL, btnR;         // butoane apasate
  bool startSig;           // modul: pin Start = 1
  bool killSig;            // modul: pin Kill  = 0 (=> stop)
};

struct CalData { uint8_t magic; uint16_t thr; uint8_t whiteIsLow; uint8_t check; };
#define CAL_MAGIC 0xB7

// prototipuri (functiile primesc tipuri definite mai sus)
static void readSensors(Sensors &s);
static void idleStep(const Sensors &s, unsigned long now);
static void countdownStep(const Sensors &s, unsigned long now);
static void fightLoop(const Sensors &s, unsigned long now);
static void fightStep(const Sensors &s, unsigned long now);
static void attackMotors(const Sensors &s);
static void printTelemetry(const Sensors &s);
static uint8_t calChecksum(const CalData &c);

static MainState  mainState = ST_IDLE;
static Tactic     tactic    = TACTIC_STRAIGHT;
static bool       testMode  = false;
static bool       telemetry = false;
static uint16_t   lineThreshold  = LINE_THRESHOLD_DEFAULT;
static bool       lineWhiteIsLow = (LINE_WHITE_IS_LOW_DEFAULT != 0);

// butoane in asteptare
static bool          btnWasPressed = false;
static unsigned long btnPressStart = 0;
static uint8_t       btnMask       = 0;      // bit0 = stanga, bit1 = dreapta
static bool          btnConsumed   = false;
static bool          waitRelease   = true;   // ignoram butoanele pana sunt eliberate
static unsigned long releaseSince  = 0;
static unsigned long tacticShowUntil = 0;

// numaratoare
static unsigned long countdownStart = 0;
static bool          cdReleased     = false;
static unsigned long cdReleaseAt    = 0;
static unsigned long cdPressSince   = 0;

// modul de start
#if USE_START_MODULE
static bool          moduleArmed    = false;
static bool          modLast        = false;
static unsigned long modChangeAt    = 0;
static bool          modHighStable  = false;
static bool          modLowStable   = false;
static bool          fightByModule  = false;
static bool          moduleSeenHigh = false;
#endif
#if USE_KILL_PIN
static bool          killLast       = false;
static unsigned long killChangeAt   = 0;
static bool          killLowStable  = false;
#endif

// lupta
static FightState    fs             = FS_SEARCH_SPIN;
static unsigned long fsStart        = 0;
static int8_t        lastDir        = DIR_LEFT;   // ultima parte unde a fost vazut adversarul
static unsigned long lastSeen       = 0;          // 0 = nevazut inca in lupta asta
static int8_t        searchDir      = DIR_LEFT;
static unsigned long searchSpinMs   = SEARCH_SPIN_MS;
static int8_t        edgeTurnDir    = DIR_LEFT;
static int8_t        lastEdgeTurnDir = DIR_RIGHT;
static unsigned long stopHoldSince  = 0;
static unsigned long pushStart      = 0;
static unsigned long lastTelemetryAt = 0;

// =====================================================================
//  4. LED-URI SI MOTOARE
// =====================================================================
static inline void leds(bool a, bool b) {
  digitalWrite(PIN_LED1, a ? HIGH : LOW);
  digitalWrite(PIN_LED2, b ? HIGH : LOW);
}

// Un canal de motor, schema din codul oficial JSumo:
//   inainte : DIR=HIGH, PWM = 255 - viteza
//   inapoi  : DIR=LOW,  PWM = viteza
//   oprit   : DIR=HIGH, PWM = 255  (frana)
static void driveChannel(uint8_t dirPin, uint8_t pwmPin, int v) {
  if (v >  255) v =  255;
  if (v < -255) v = -255;
  if (v > 0) {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, 255 - v);
  } else if (v < 0) {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, -v);
  } else {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, 255);
  }
}

// setMotors(stanga, dreapta): -255..255, pozitiv = inainte
static void setMotors(int left, int right) {
#if MOTOR_LEFT_INVERT
  left = -left;
#endif
#if MOTOR_RIGHT_INVERT
  right = -right;
#endif
#if MOTOR_SWAP
  driveChannel(PIN_MR_DIR, PIN_MR_PWM, left);
  driveChannel(PIN_ML_DIR, PIN_ML_PWM, right);
#else
  driveChannel(PIN_ML_DIR, PIN_ML_PWM, left);
  driveChannel(PIN_MR_DIR, PIN_MR_PWM, right);
#endif
}

static inline void motorsBrake() { setMotors(0, 0); }

// rotire pe loc: DIR_LEFT = roata stanga inapoi + dreapta inainte
static inline void pivot(int8_t dir, int speed) {
  if (dir == DIR_LEFT) setMotors(-speed, speed);
  else                 setMotors(speed, -speed);
}

// =====================================================================
//  5. SENZORI
// =====================================================================
static inline bool lineIsWhite(int v) {
  return lineWhiteIsLow ? (v < (int)lineThreshold) : (v > (int)lineThreshold);
}

static void readSensors(Sensors &s) {
  bool l = digitalRead(PIN_OPP_LEFT)  == HIGH;
  bool f = digitalRead(PIN_OPP_FRONT) == HIGH;
  bool r = digitalRead(PIN_OPP_RIGHT) == HIGH;
#if OPP_ACTIVE_HIGH
  s.oppL = l;  s.oppF = f;  s.oppR = r;
#else
  s.oppL = !l; s.oppF = !f; s.oppR = !r;
#endif

  s.lineRaw = analogRead(PIN_LINE);
  s.line = lineIsWhite(s.lineRaw);
  for (uint8_t i = 1; s.line && i < LINE_CONFIRM_SAMPLES; i++) {   // confirmare (~0,1 ms/citire)
    s.line = lineIsWhite(analogRead(PIN_LINE));
  }

  s.btnL = digitalRead(PIN_BTN_LEFT)  == LOW;
  s.btnR = digitalRead(PIN_BTN_RIGHT) == LOW;

#if USE_START_MODULE
  s.startSig = digitalRead(PIN_START_MODULE) == HIGH;
#else
  s.startSig = false;
#endif
#if USE_KILL_PIN
  s.killSig = digitalRead(PIN_KILL_MODULE) == LOW;
#else
  s.killSig = false;
#endif
}

static bool anyButton() {
  return digitalRead(PIN_BTN_LEFT) == LOW || digitalRead(PIN_BTN_RIGHT) == LOW;
}

static void waitButtonsReleased() {
  while (anyButton()) delay(5);
  delay(150);
}

// =====================================================================
//  6. CALIBRARE LINIE (EEPROM)
// =====================================================================
static uint8_t calChecksum(const CalData &c) {
  return (uint8_t)(c.magic ^ (c.thr & 0xFF) ^ (c.thr >> 8) ^ c.whiteIsLow ^ 0x5A);
}

static void loadCalibration() {
  CalData c;
  EEPROM.get(0, c);
  if (c.magic == CAL_MAGIC && c.check == calChecksum(c) && c.thr > 0 && c.thr < 1023) {
    lineThreshold  = c.thr;
    lineWhiteIsLow = (c.whiteIsLow != 0);
  }
}

static void saveCalibration() {
  CalData c;
  c.magic = CAL_MAGIC;
  c.thr = lineThreshold;
  c.whiteIsLow = lineWhiteIsLow ? 1 : 0;
  c.check = calChecksum(c);
  EEPROM.put(0, c);
}

static void clearCalibration() {
  EEPROM.write(0, 0xFF);
  lineThreshold  = LINE_THRESHOLD_DEFAULT;
  lineWhiteIsLow = (LINE_WHITE_IS_LOW_DEFAULT != 0);
}

static int sampleLine(uint8_t n) {
  long sum = 0;
  for (uint8_t i = 0; i < n; i++) { sum += analogRead(PIN_LINE); delay(5); }
  return (int)(sum / n);
}

static void blinkBoth(uint8_t times, unsigned int ms) {
  for (uint8_t i = 0; i < times; i++) { leds(true, true); delay(ms); leds(false, false); delay(ms); }
}

// Asteapta o apasare de buton; intre timp LED1 clipeste daca blinkLed1, altfel ambele stau aprinse.
static void waitPressWithLeds(bool blinkLed1) {
  waitButtonsReleased();
  while (!anyButton()) {
    if (blinkLed1) leds(((millis() / 150) & 1) != 0, false);
    else           leds(true, true);
    delay(5);
  }
  delay(BTN_DEBOUNCE_MS);
  waitButtonsReleased();
}

static void calibrateLine() {
  motorsBrake();
  if (Serial) Serial.println(F("[CAL] Aseaza robotul pe dohyo cu senzorul de linie pe NEGRU si apasa un buton..."));
  waitPressWithLeds(false);                 // ambele LED-uri aprinse = asteptam NEGRU
  int black = sampleLine(64);
  if (Serial) { Serial.print(F("[CAL] negru = ")); Serial.println(black); }

  if (Serial) Serial.println(F("[CAL] Pune senzorul de linie pe LINIA ALBA si apasa un buton..."));
  waitPressWithLeds(true);                  // LED1 clipeste = asteptam ALB
  int white = sampleLine(64);
  if (Serial) { Serial.print(F("[CAL] alb = ")); Serial.println(white); }

  int diff = abs(black - white);
  if (diff < 40) {                          // contrast prea mic: senzor murdar/neconectat/prea sus
    if (Serial) Serial.println(F("[CAL] EROARE: diferenta prea mica intre alb si negru. Pastrez valorile vechi."));
    blinkBoth(10, 80);
    return;
  }
  lineWhiteIsLow = (white < black);
  // pragul la 35% din diferenta, pornind de la alb: prinde linia si la viteza, ignora zgarieturile
  int thr = lineWhiteIsLow ? (white + (diff * 35) / 100) : (white - (diff * 35) / 100);
  if (thr < 1) thr = 1;
  if (thr > 1022) thr = 1022;
  lineThreshold = (uint16_t)thr;
  saveCalibration();
  if (Serial) {
    Serial.print(F("[CAL] OK. prag = ")); Serial.print(lineThreshold);
    Serial.println(lineWhiteIsLow ? F(" (alb = valoare mica)") : F(" (alb = valoare mare)"));
  }
  blinkBoth(3, 200);
}

// =====================================================================
//  7. TEST MOTOARE  (tine robotul in aer!)
// =====================================================================
static void motorStep(const __FlashStringHelper *name, int l, int r) {
  if (Serial) { Serial.print(F("[TEST] ")); Serial.println(name); }
  leds(l != 0, r != 0);                     // LED1 = roata stanga, LED2 = roata dreapta
  setMotors(l, r);
  delay(700);
  motorsBrake();
  leds(false, false);
  delay(400);
}

static void motorTest() {
  if (Serial) Serial.println(F("[TEST] Test motoare in 1 s - tine robotul in aer!"));
  blinkBoth(5, 100);
  motorStep(F("1/8 roata STANGA inainte"),      150,    0);
  motorStep(F("2/8 roata STANGA inapoi"),      -150,    0);
  motorStep(F("3/8 roata DREAPTA inainte"),       0,  150);
  motorStep(F("4/8 roata DREAPTA inapoi"),        0, -150);
  motorStep(F("5/8 ambele inainte"),            150,  150);
  motorStep(F("6/8 ambele inapoi"),            -150, -150);
  motorStep(F("7/8 rotire STANGA pe loc"),     -150,  150);
  motorStep(F("8/8 rotire DREAPTA pe loc"),     150, -150);
  if (Serial) Serial.println(F("[TEST] gata"));
}

// =====================================================================
//  8. MODUL DE START (debounce + armare)
// =====================================================================
#if USE_START_MODULE
static void moduleUpdate(const Sensors &s, unsigned long now) {
  if (s.startSig != modLast) {
    modLast = s.startSig;
    modChangeAt = now;
    modHighStable = false;
    modLowStable  = false;
  } else if (now - modChangeAt >= MODULE_DEBOUNCE_MS) {
    modHighStable = modLast;
    modLowStable  = !modLast;
  }
  // acceptam un START doar dupa ce am vazut modulul stabil in "0" (asa cere regulamentul la pornire)
  if (modLowStable && now - modChangeAt >= MODULE_ARM_MS) moduleArmed = true;
}
#endif

#if USE_KILL_PIN
static void killUpdate(const Sensors &s, unsigned long now) {
  if (s.killSig != killLast) {
    killLast = s.killSig;
    killChangeAt = now;
    killLowStable = false;
  } else if (now - killChangeAt >= MODULE_DEBOUNCE_MS) {
    killLowStable = killLast;
  }
}
#endif

// =====================================================================
//  9. TRANZITII PRINCIPALE
// =====================================================================
static const __FlashStringHelper *tacticName() {
  switch (tactic) {
    case TACTIC_LEFT:  return F("STANGA");
    case TACTIC_RIGHT: return F("DREAPTA");
    default:           return F("DREPT");
  }
}

static const __FlashStringHelper *stateName() {
  if (mainState == ST_COUNTDOWN) return F("NUMARATOARE");
  if (mainState == ST_IDLE)      return F("ASTEPTARE");
  switch (fs) {
    case FS_SEARCH_SPIN: return F("LUPTA/cauta-rotire");
    case FS_SEARCH_FWD:  return F("LUPTA/cauta-inainte");
    case FS_ATTACK:      return F("LUPTA/ATAC");
    case FS_EDGE_BACK:   return F("LUPTA/margine-inapoi");
    default:             return F("LUPTA/margine-rotire");
  }
}

static Tactic tacticFromMask(uint8_t m) {
  if (m == 3)  return TACTIC_STRAIGHT;
  if (m & 2)   return TACTIC_RIGHT;
  return TACTIC_LEFT;
}

static inline void enterFightState(FightState st, unsigned long now) {
  fs = st;
  fsStart = now;
  if (st == FS_SEARCH_SPIN) searchSpinMs = SEARCH_SPIN_MS;
}

static void startFight(bool byModule, unsigned long now) {
  mainState = ST_FIGHT;
#if USE_START_MODULE
  fightByModule  = byModule;
  moduleSeenHigh = byModule;
  moduleArmed    = false;
#else
  (void)byModule;
#endif
  stopHoldSince = 0;
  lastSeen  = 0;
  pushStart = 0;
  switch (tactic) {
    case TACTIC_LEFT:
      searchDir = DIR_LEFT;  lastDir = DIR_LEFT;
      enterFightState(FS_SEARCH_SPIN, now); searchSpinMs = INITIAL_SPIN_MS;
      break;
    case TACTIC_RIGHT:
      searchDir = DIR_RIGHT; lastDir = DIR_RIGHT;
      enterFightState(FS_SEARCH_SPIN, now); searchSpinMs = INITIAL_SPIN_MS;
      break;
    default:
      searchDir = DIR_LEFT;
      enterFightState(FS_SEARCH_FWD, now);
      break;
  }
  if (Serial) { Serial.print(F("[LUPTA] START, tactica ")); Serial.println(tacticName()); }
}

static void stopFight() {
  motorsBrake();
  mainState = ST_IDLE;
  waitRelease = true;
  releaseSince = 0;
#if USE_START_MODULE
  moduleArmed = false;
#endif
  leds(false, false);
  if (Serial) Serial.println(F("[LUPTA] STOP"));
}

static void startCountdown(unsigned long t0, unsigned long now) {
  motorsBrake();
  if (START_DELAY_MS == 0) { startFight(false, now); return; }
  mainState = ST_COUNTDOWN;
  countdownStart = t0;
  cdReleased = false;
  cdReleaseAt = 0;
  cdPressSince = 0;
  if (Serial) { Serial.print(F("[NUMARATOARE] tactica ")); Serial.println(tacticName()); }
}

// =====================================================================
// 10. STARI: ASTEPTARE / NUMARATOARE / LUPTA
// =====================================================================
static void idleStep(const Sensors &s, unsigned long now) {
  const bool pressed = s.btnL || s.btnR;

  // LED-uri: tactica aleasa (1 s dupa apasare), altfel oglinda senzorilor
  if ((long)(tacticShowUntil - now) > 0) leds(tactic != TACTIC_RIGHT, tactic != TACTIC_LEFT);
  else                                   leds(s.oppL || s.oppF || s.oppR, s.line);

#if USE_START_MODULE
  if (moduleArmed && modHighStable) { startFight(true, now); return; }   // arbitrul a dat START
#endif

  // dupa un stop / la pornire: asteptam eliberarea butoanelor
  if (waitRelease) {
    if (pressed) { releaseSince = 0; return; }
    if (releaseSince == 0) { releaseSince = now; return; }
    if (now - releaseSince < 200) return;
    waitRelease = false;
    btnWasPressed = false;
  }

  if (pressed) {
    if (!btnWasPressed) { btnWasPressed = true; btnPressStart = now; btnMask = 0; btnConsumed = false; }
    btnMask |= (s.btnL ? 1 : 0) | (s.btnR ? 2 : 0);
    const unsigned long held = now - btnPressStart;
    if (!btnConsumed && held >= BTN_DEBOUNCE_MS) {
      if (testMode) {
        btnConsumed = true;
        if (btnMask & 1) motorTest(); else calibrateLine();
        waitRelease = true; releaseSince = 0;
      }
#if USE_START_MODULE
      else if (held >= LONG_PRESS_MS) {            // pornire de test, fara telecomanda
        btnConsumed = true;
        tactic = tacticFromMask(btnMask);
        startCountdown(now, now);
      }
#endif
    }
  } else if (btnWasPressed) {                      // eliberare
    btnWasPressed = false;
    const unsigned long held = now - btnPressStart;
    if (!btnConsumed && held >= BTN_DEBOUNCE_MS) {
      tactic = tacticFromMask(btnMask);
      tacticShowUntil = now + 1000;
      if (Serial) { Serial.print(F("[TACTICA] ")); Serial.println(tacticName()); }
#if !USE_START_MODULE
      startCountdown(btnPressStart, now);          // numaratoarea curge de la apasare
#endif
    }
  }
}

static void countdownStep(const Sensors &s, unsigned long now) {
  const unsigned long elapsed = now - countdownStart;
  const bool pressed = s.btnL || s.btnR;

  // anulare: butoanele au fost eliberate, apoi apasate din nou
  if (!pressed) {
    if (!cdReleased) { cdReleased = true; cdReleaseAt = now; }
    cdPressSince = 0;
  } else if (cdReleased && now - cdReleaseAt >= 150) {
    if (cdPressSince == 0) cdPressSince = now;
    else if (now - cdPressSince >= BTN_DEBOUNCE_MS) {
      motorsBrake();
      mainState = ST_IDLE;
      waitRelease = true; releaseSince = 0;
      leds(false, false);
      if (Serial) Serial.println(F("[NUMARATOARE] anulata"));
      return;
    }
  }

#if USE_START_MODULE
  if (moduleArmed && modHighStable) { startFight(true, now); return; }
#endif

  // LED-uri alternante, mai rapid in ultima secunda
  const unsigned long remaining = (elapsed < START_DELAY_MS) ? (START_DELAY_MS - elapsed) : 0;
  const unsigned long period = (remaining < 1000) ? 100 : 250;
  const bool phase = ((elapsed / period) & 1) != 0;
  leds(phase, !phase);

  if (elapsed >= START_DELAY_MS) startFight(false, now);
}

static void fightLoop(const Sensors &s, unsigned long now) {
  // ---- conditii de STOP
  if (s.btnL || s.btnR) {
    if (stopHoldSince == 0) stopHoldSince = now;
    else if (now - stopHoldSince >= STOP_HOLD_MS) { stopFight(); return; }
  } else {
    stopHoldSince = 0;
  }
#if USE_START_MODULE
  if (modHighStable) moduleSeenHigh = true;
  if ((fightByModule || moduleSeenHigh) && modLowStable) { stopFight(); return; }
#endif
#if USE_KILL_PIN
  if (killLowStable) { stopFight(); return; }
#endif

  fightStep(s, now);
  leds(s.oppL || s.oppF || s.oppR, s.line);
}

// comanda de atac in functie de combinatia de senzori
static void attackMotors(const Sensors &s) {
  if (s.oppF) {
    if (s.oppL && !s.oppR)      setMotors(SPEED_ATTACK_ARC, SPEED_ATTACK);   // fata + stanga: arc spre stanga
    else if (s.oppR && !s.oppL) setMotors(SPEED_ATTACK, SPEED_ATTACK_ARC);   // fata + dreapta: arc spre dreapta
    else                        setMotors(SPEED_ATTACK, SPEED_ATTACK);       // drept in el
  } else if (s.oppL && !s.oppR) {
    pivot(DIR_LEFT, SPEED_TURN);
  } else if (s.oppR && !s.oppL) {
    pivot(DIR_RIGHT, SPEED_TURN);
  } else {
    setMotors(SPEED_ATTACK, SPEED_ATTACK);                                   // stanga + dreapta (foarte aproape)
  }
}

static void fightStep(const Sensors &s, unsigned long now) {
  const bool any = s.oppL || s.oppF || s.oppR;
  if (any) {
    lastSeen = now;
    if (s.oppL && !s.oppR)      lastDir = DIR_LEFT;
    else if (s.oppR && !s.oppL) lastDir = DIR_RIGHT;
  }

  // ---- 1. MARGINEA (prioritate maxima)
  if (s.line && fs != FS_EDGE_BACK) {
#if EDGE_PUSH_OVERRIDE_MS > 0
    if (s.oppF) {
      if (pushStart == 0) pushStart = now;
      if (now - pushStart < EDGE_PUSH_OVERRIDE_MS) { setMotors(SPEED_ATTACK, SPEED_ATTACK); return; }
    }
#endif
    // ne intoarcem spre ultima pozitie a adversarului; daca nu l-am vazut recent, alternam
    if (lastSeen != 0 && now - lastSeen < 1500) edgeTurnDir = lastDir;
    else edgeTurnDir = (lastEdgeTurnDir == DIR_LEFT) ? DIR_RIGHT : DIR_LEFT;
    lastEdgeTurnDir = edgeTurnDir;
    enterFightState(FS_EDGE_BACK, now);
    setMotors(-SPEED_EDGE_BACK, -SPEED_EDGE_BACK);
    return;
  }
  if (!s.line) pushStart = 0;

  // ---- 2. restul comportamentului
  switch (fs) {
    case FS_EDGE_BACK:
      setMotors(-SPEED_EDGE_BACK, -SPEED_EDGE_BACK);
      if (now - fsStart >= EDGE_BACK_MS) enterFightState(FS_EDGE_TURN, now);
      break;

    case FS_EDGE_TURN:
      if (any && now - fsStart >= EDGE_TURN_MIN_MS) {
        enterFightState(FS_ATTACK, now);
        attackMotors(s);
        break;
      }
      pivot(edgeTurnDir, SPEED_EDGE_TURN);
      if (now - fsStart >= EDGE_TURN_MS) { searchDir = edgeTurnDir; enterFightState(FS_SEARCH_SPIN, now); }
      break;

    case FS_ATTACK:
      if (any) attackMotors(s);
      else if (now - lastSeen >= OPP_LOST_MS) { searchDir = lastDir; enterFightState(FS_SEARCH_SPIN, now); }
      // altfel pastram ultima comanda cateva ms (filtru pentru "clipirea" senzorilor)
      break;

    case FS_SEARCH_SPIN:
      if (any) { enterFightState(FS_ATTACK, now); attackMotors(s); break; }
      pivot(searchDir, SPEED_SEARCH_SPIN);
      if (now - fsStart >= searchSpinMs) enterFightState(FS_SEARCH_FWD, now);
      break;

    case FS_SEARCH_FWD:
      if (any) { enterFightState(FS_ATTACK, now); attackMotors(s); break; }
      setMotors(SPEED_SEARCH_FWD, SPEED_SEARCH_FWD);
      if (now - fsStart >= SEARCH_FWD_MS) { searchDir = lastDir; enterFightState(FS_SEARCH_SPIN, now); }
      break;
  }
}

// =====================================================================
// 11. SERIAL (optional, doar cu USB)
// =====================================================================
static void printHelp() {
  Serial.println(F("BauBau Micro Sumo - comenzi:"));
  Serial.println(F("  t = telemetrie on/off   m = test motoare (robotul in aer!)"));
  Serial.println(F("  c = calibrare linie     d = prag linie implicit"));
  Serial.println(F("  1/2/3 = tactica stanga/drept/dreapta"));
  Serial.println(F("  s = START imediat (robotul in aer!)   x = STOP"));
}

static void printTelemetry(const Sensors &s) {
  Serial.print(F("ADV L:")); Serial.print(s.oppL);
  Serial.print(F(" F:"));    Serial.print(s.oppF);
  Serial.print(F(" R:"));    Serial.print(s.oppR);
  Serial.print(F(" | LINIE raw:")); Serial.print(s.lineRaw);
  Serial.print(F(" prag:"));        Serial.print(lineThreshold);
  Serial.print(s.line ? F(" ALB") : F(" negru"));
  Serial.print(F(" | BTN L:")); Serial.print(s.btnL);
  Serial.print(F(" R:"));       Serial.print(s.btnR);
#if USE_START_MODULE
  Serial.print(F(" | START:")); Serial.print(s.startSig);
#endif
#if USE_KILL_PIN
  Serial.print(F(" KILL:"));    Serial.print(s.killSig);
#endif
  Serial.print(F(" | ")); Serial.print(stateName());
  Serial.print(F(" tactica:")); Serial.println(tacticName());
}

static void handleSerial(unsigned long now) {
  const char c = (char)Serial.read();
  switch (c) {
    case 't': telemetry = !telemetry; Serial.println(telemetry ? F("[SER] telemetrie ON") : F("[SER] telemetrie OFF")); break;
    case 'm': if (mainState == ST_IDLE) motorTest(); break;
    case 'c': if (mainState == ST_IDLE) calibrateLine(); break;
    case 'd': clearCalibration(); Serial.println(F("[SER] prag linie resetat la valoarea implicita")); break;
    case '1': tactic = TACTIC_LEFT;     Serial.println(F("[SER] tactica STANGA"));  break;
    case '2': tactic = TACTIC_STRAIGHT; Serial.println(F("[SER] tactica DREPT"));   break;
    case '3': tactic = TACTIC_RIGHT;    Serial.println(F("[SER] tactica DREAPTA")); break;
    case 's': if (mainState == ST_IDLE) { Serial.println(F("[SER] START imediat")); startFight(false, now); } break;
    case 'x': if (mainState != ST_IDLE) stopFight(); break;
    case '?': case 'h': printHelp(); break;
    default: break;   // ignoram \r, \n etc.
  }
}

// =====================================================================
// 12. SETUP / LOOP
// =====================================================================
void setup() {
  // 1. motoarele oprite inainte de orice altceva
  pinMode(PIN_ML_DIR, OUTPUT); pinMode(PIN_ML_PWM, OUTPUT);
  pinMode(PIN_MR_DIR, OUTPUT); pinMode(PIN_MR_PWM, OUTPUT);
  motorsBrake();
#if PWM_HIGH_FREQ
  TCCR1B = (TCCR1B & 0xF8) | 0x01;   // Timer1 (pin 9)  -> ~31 kHz
  TCCR3B = (TCCR3B & 0xF8) | 0x01;   // Timer3 (pin 5)  -> ~31 kHz
#endif

  pinMode(PIN_LED1, OUTPUT); pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_BTN_LEFT,  INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
  pinMode(PIN_OPP_LEFT,  INPUT_PULLUP);
  pinMode(PIN_OPP_FRONT, INPUT_PULLUP);
  pinMode(PIN_OPP_RIGHT, INPUT_PULLUP);
  pinMode(PIN_LINE, INPUT);
#if USE_START_MODULE
  pinMode(PIN_START_MODULE, INPUT);
#endif
#if USE_KILL_PIN
  pinMode(PIN_KILL_MODULE, INPUT);
#endif

  Serial.begin(SERIAL_BAUD);           // NU asteptam USB-ul: robotul merge si fara cablu
  loadCalibration();

  // 2. moduri speciale: buton tinut apasat la pornire
  delay(50);
  const bool holdL = digitalRead(PIN_BTN_LEFT)  == LOW;
  const bool holdR = digitalRead(PIN_BTN_RIGHT) == LOW;
  if (holdL) {
    testMode  = true;
    telemetry = true;
    blinkBoth(3, 100);                 // 3 clipiri = mod TEST
    waitButtonsReleased();
  } else if (holdR) {
    waitButtonsReleased();
    calibrateLine();
  } else {
    leds(true, false); delay(120);     // "salut" scurt
    leds(false, true); delay(120);
    leds(false, false);
  }
  waitRelease = true;
  releaseSince = 0;
}

void loop() {
  const unsigned long now = millis();
  Sensors s;
  readSensors(s);

#if USE_START_MODULE
  moduleUpdate(s, now);
#endif
#if USE_KILL_PIN
  killUpdate(s, now);
#endif
  if (Serial.available()) handleSerial(now);

  switch (mainState) {
    case ST_IDLE:      idleStep(s, now);      break;
    case ST_COUNTDOWN: countdownStep(s, now); break;
    case ST_FIGHT:     fightLoop(s, now);     break;
  }

  if (telemetry && Serial && now - lastTelemetryAt >= 100) {
    lastTelemetryAt = now;
    printTelemetry(s);
  }
}
