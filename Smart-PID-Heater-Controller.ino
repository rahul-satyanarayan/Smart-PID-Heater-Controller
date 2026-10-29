/*
 * ═══════════════════════════════════════════════════════════════════════════
 * 180W / 24V Smart Heater Controller — Modular Hardware Configuration Edition
 * MCU: Arduino (Uno/Nano/Mega)
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <U8g2lib.h>
#include <max6675.h>
#include <EEPROM.h>
#include <Encoder.h>

// ── Hardware Pin Configuration ────────────────────────────────────────────────
const uint8_t PIN_HEATER_A  = 10;
const uint8_t PIN_HEATER_B  = 11;
const uint8_t PIN_SWITCH    =  2;
const uint8_t PIN_SWITCH_2  =  8;
const uint8_t PIN_ENC_CLK   =  3;
const uint8_t PIN_ENC_DT    =  4;
const uint8_t PIN_TC_SO     =  5;
const uint8_t PIN_TC_CS     =  6;
const uint8_t PIN_TC_SCK    =  7;
const uint8_t PIN_BATT_MON  = A2;

// ── USER-CONFIGURABLE HARDWARE PROFILE ────────────────────────────────────────
const float   RESISTOR_1    = 10000.0; // R1 value in Ohms (Connected to Batt +)
const float   RESISTOR_2    = 2200.0;  // R2 value in Ohms (Connected to GND)

// 24V Lead-Acid Pack Voltage Window
// Adjust these once you have the battery datasheet:
//   Flooded : MIN=21.6  MAX=27.6
//   AGM     : MIN=21.6  MAX=28.0
//   Gel     : MIN=21.6  MAX=27.2
const float   PACK_MIN_V    = 21.6;    // 1.80V/cell x 12 cells (fully discharged)
const float   PACK_MAX_V    = 27.6;    // 2.30V/cell x 12 cells (fully charged)

// Arduino ADC Reference Voltage (Standard VCC or precise regulated 5.0V rail)
const float   MCU_VCC       = 5.0;

// ── EEPROM Address Map ────────────────────────────────────────────────────────
#define ADDR_SET_TEMP   0
#define ADDR_KP         4
#define ADDR_KI         8
#define ADDR_KD         12
#define ADDR_MANUAL_FLG 16

// ── Boundary Safety Configurations ────────────────────────────────────────────
const double TEMP_MIN = 0.0,   TEMP_MAX = 500.0;
const double OUT_MIN  = 0.0,   OUT_MAX  = 255.0;

#define TC_INTERVAL_MS    250UL
#define PID_INTERVAL_MS   200UL
#define BTN_DEBOUNCE_MS   250UL
#define LONG_PRESS_MS    3000UL

struct PIDGains { double Kp, Ki, Kd; };

PIDGains getPIDForTemp(double temp) {
    if      (temp <  25.0) return { 5.00, 0.050, 2.0 };
    else if (temp <  50.0) return { 5.00, 0.048, 2.2 };
    else if (temp <  75.0) return { 4.80, 0.046, 2.5 };
    else if (temp < 100.0) return { 4.80, 0.044, 2.8 };
    else if (temp < 125.0) return { 4.20, 0.020, 3.8 };
    else if (temp < 150.0) return { 4.00, 0.018, 4.2 };
    else if (temp < 175.0) return { 3.90, 0.016, 4.5 };
    else if (temp < 200.0) return { 3.70, 0.015, 4.8 };
    else if (temp < 225.0) return { 3.60, 0.014, 5.0 };
    else if (temp < 250.0) return { 3.50, 0.013, 5.2 };
    else if (temp < 275.0) return { 4.50, 0.045, 4.0 };
    else if (temp < 300.0) return { 4.80, 0.055, 3.8 };
    else if (temp < 325.0) return { 5.20, 0.065, 3.5 };
    else if (temp < 350.0) return { 5.50, 0.075, 3.2 };
    else if (temp < 375.0) return { 5.80, 0.080, 3.0 };
    else if (temp < 400.0) return { 6.00, 0.085, 2.8 };
    else if (temp < 425.0) return { 6.20, 0.090, 2.5 };
    else if (temp < 450.0) return { 6.50, 0.095, 2.2 };
    else if (temp < 475.0) return { 6.80, 0.100, 2.0 };
    else                   return { 7.00, 0.110, 1.8 };
}

// ── Core Control Variables ────────────────────────────────────────────────────
double setTemp    = 150.0;
double inputTemp  = 0.0;
double outputPWM  = 0.0;

double Kp = 4.50, Ki = 0.040, Kd = 3.5;
bool manualPIDMode = false;

// Calculated from resistor config in setup()
float DIVIDER_RATIO = 0.0;

float trueBatteryVoltage = 0.0;
float batteryPercentage  = 0.0;
bool isBatteryPower      = false;

double integralTerm    = 0.0;
double prevMeasurement = 0.0;
unsigned long lastPIDMs    = 0;
unsigned long lastTCReadMs = 0;

enum SystemMode : uint8_t {
    MODE_RUNNING = 0,
    MODE_SET_TEMP,
    MODE_EDIT_KP,
    MODE_EDIT_KI,
    MODE_EDIT_KD
};
SystemMode currentMode = MODE_RUNNING;

U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
MAX6675 thermocouple(PIN_TC_SCK, PIN_TC_CS, PIN_TC_SO);
Encoder myEnc(PIN_ENC_CLK, PIN_ENC_DT);

long oldEncoderPos = -999;

// ── Prototypes ────────────────────────────────────────────────────────────────
void loadAllSystemMemory();
void saveAllSystemMemory();
bool isValidDouble(double v, double lo, double hi);
void checkPowerSource();
void handleButton();
void handleButton2();
void handleEncoder();
void computePID();
void resetPIDState();
void updateDisplay();

// ═════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);

    // Only DIVIDER_RATIO is derived now; pack limits are fixed constants above
    DIVIDER_RATIO = (RESISTOR_1 + RESISTOR_2) / RESISTOR_2;

    pinMode(PIN_SWITCH,   INPUT_PULLUP);
    pinMode(PIN_SWITCH_2, INPUT_PULLUP);
    pinMode(PIN_HEATER_A, OUTPUT);
    pinMode(PIN_HEATER_B, OUTPUT);
    analogWrite(PIN_HEATER_A, 0);
    analogWrite(PIN_HEATER_B, 0);

    u8g2.begin();
    loadAllSystemMemory();

    if (!manualPIDMode) {
        PIDGains g = getPIDForTemp(setTemp);
        Kp = g.Kp; Ki = g.Ki; Kd = g.Kd;
    }

    delay(500);
    inputTemp       = thermocouple.readCelsius();
    prevMeasurement = inputTemp;
    lastTCReadMs    = millis();
    lastPIDMs       = millis();
    currentMode     = MODE_RUNNING;
}

// ═════════════════════════════════════════════════════════════════════════════
void loop() {
    if (millis() - lastTCReadMs >= TC_INTERVAL_MS) {
        inputTemp    = thermocouple.readCelsius();
        lastTCReadMs = millis();
        checkPowerSource();
    }

    handleButton();
    handleButton2();
    handleEncoder();

    if (currentMode == MODE_RUNNING) {
        computePID();
        analogWrite(PIN_HEATER_A, (int)outputPWM);
        analogWrite(PIN_HEATER_B, (int)outputPWM);
    } else {
        analogWrite(PIN_HEATER_A, 0);
        analogWrite(PIN_HEATER_B, 0);
        outputPWM       = 0;
        lastPIDMs       = millis();
        prevMeasurement = inputTemp;
    }

    updateDisplay();
}

// ═════════════════════════════════════════════════════════════════════════════
void computePID() {
    unsigned long now = millis();
    double dt = (double)(now - lastPIDMs) / 1000.0;
    if (dt < ((double)PID_INTERVAL_MS / 1000.0)) return;
    lastPIDMs = now;

    double error    = setTemp - inputTemp;
    double dMeasure = inputTemp - prevMeasurement;
    prevMeasurement = inputTemp;

    if (abs(error) <= 12.0) {
        integralTerm += Ki * error * dt;
        integralTerm  = constrain(integralTerm, OUT_MIN, OUT_MAX);
    } else {
        integralTerm = 0;
    }

    double dTerm = -Kd * (dMeasure / dt);
    outputPWM = constrain((Kp * error) + integralTerm + dTerm, OUT_MIN, OUT_MAX);
}

void resetPIDState() {
    integralTerm    = 0.0;
    prevMeasurement = inputTemp;
    lastPIDMs       = millis();
}

// ── Button 1 Logic ────────────────────────────────────────────────────────────
void handleButton() {
    static unsigned long pressedAt = 0;
    static bool isDown = false;

    bool pressed = (digitalRead(PIN_SWITCH) == LOW);
    if (pressed && !isDown) { isDown = true; pressedAt = millis(); }

    if (pressed && isDown && (millis() - pressedAt >= LONG_PRESS_MS)) {
        saveAllSystemMemory();
        resetPIDState();
        currentMode = MODE_RUNNING;
        isDown = false;
        return;
    }

    if (!pressed && isDown) {
        unsigned long held = millis() - pressedAt;
        isDown = false;

        if (held >= BTN_DEBOUNCE_MS && held < LONG_PRESS_MS) {
            if (currentMode == MODE_RUNNING) {
                currentMode = MODE_SET_TEMP;
            } else {
                resetPIDState();
                currentMode = MODE_RUNNING;
            }
        }
    }
}

// ── Button 2 Logic ────────────────────────────────────────────────────────────
void handleButton2() {
    static unsigned long pressedAt2 = 0;
    static bool isDown2 = false;

    if (currentMode == MODE_RUNNING) return;

    bool pressed2 = (digitalRead(PIN_SWITCH_2) == LOW);
    if (pressed2 && !isDown2) { isDown2 = true; pressedAt2 = millis(); }

    if (pressed2 && isDown2 && (millis() - pressedAt2 >= LONG_PRESS_MS)) {
        isDown2 = false;

        if (currentMode == MODE_SET_TEMP)      currentMode = MODE_EDIT_KP;
        else if (currentMode == MODE_EDIT_KP)  currentMode = MODE_EDIT_KI;
        else if (currentMode == MODE_EDIT_KI)  currentMode = MODE_EDIT_KD;
        else if (currentMode == MODE_EDIT_KD)  currentMode = MODE_SET_TEMP;

        manualPIDMode = true;
        delay(200);
    }
    if (!pressed2) isDown2 = false;
}

// ── Encoder Processing ────────────────────────────────────────────────────────
void handleEncoder() {
    long newPos = myEnc.read() / 2;
    if (newPos == oldEncoderPos) return;

    bool up = (newPos > oldEncoderPos);
    oldEncoderPos = newPos;

    switch (currentMode) {
        case MODE_SET_TEMP:
            setTemp = constrain(setTemp + (up ? 5.0 : -5.0), TEMP_MIN, TEMP_MAX);
            manualPIDMode = false;
            {
                PIDGains g = getPIDForTemp(setTemp);
                Kp = g.Kp; Ki = g.Ki; Kd = g.Kd;
            }
            break;
        case MODE_EDIT_KP:
            Kp = constrain(Kp + (up ? 0.10 : -0.10), 0.0, 25.0);
            break;
        case MODE_EDIT_KI:
            Ki = constrain(Ki + (up ? 0.001 : -0.001), 0.0, 2.0);
            break;
        case MODE_EDIT_KD:
            Kd = constrain(Kd + (up ? 0.10 : -0.10), 0.0, 30.0);
            break;
    }
}

// ── Adaptive Battery Core Processing ──────────────────────────────────────────
void checkPowerSource() {
    int rawADC = analogRead(PIN_BATT_MON);

    // Threshold: ~10V on battery input → ADC ≈ 366; 400 gives safe headroom
    isBatteryPower = (rawADC >= 400);

    if (isBatteryPower) {
        float adcVoltage = (rawADC * MCU_VCC) / 1023.0;
        trueBatteryVoltage = adcVoltage * DIVIDER_RATIO;

        // Map against fixed lead-acid pack window defined in config block
        batteryPercentage = ((trueBatteryVoltage - PACK_MIN_V) / (PACK_MAX_V - PACK_MIN_V)) * 100.0;
        batteryPercentage = constrain(batteryPercentage, 0.0, 100.0);
    } else {
        trueBatteryVoltage = 0.0;
        batteryPercentage  = 0.0;
    }
}

bool isValidDouble(double v, double lo, double hi) { return (!isnan(v) && !isinf(v) && v >= lo && v <= hi); }

void loadAllSystemMemory() {
    double tTemp, tKp, tKi, tKd;
    EEPROM.get(ADDR_SET_TEMP, tTemp);
    EEPROM.get(ADDR_KP, tKp);
    EEPROM.get(ADDR_KI, tKi);
    EEPROM.get(ADDR_KD, tKd);
    EEPROM.get(ADDR_MANUAL_FLG, manualPIDMode);

    setTemp = isValidDouble(tTemp, TEMP_MIN, TEMP_MAX) ? tTemp : 150.0;

    if (manualPIDMode) {
        Kp = isValidDouble(tKp, 0.0, 25.0) ? tKp : 4.0;
        Ki = isValidDouble(tKi, 0.0, 2.0)  ? tKi : 0.02;
        Kd = isValidDouble(tKd, 0.0, 30.0) ? tKd : 4.0;
    }
}

void saveAllSystemMemory() {
    EEPROM.put(ADDR_SET_TEMP,   setTemp);
    EEPROM.put(ADDR_MANUAL_FLG, manualPIDMode);
    EEPROM.put(ADDR_KP, Kp);
    EEPROM.put(ADDR_KI, Ki);
    EEPROM.put(ADDR_KD, Kd);
    Serial.println(F("EEPROM STORAGE COMMITTED"));
}

// ── Upgraded High-Tech UI Engine ──────────────────────────────────────────────
void updateDisplay() {
    double tempError = setTemp - inputTemp;

    u8g2.firstPage();
    do {
        u8g2.setFont(u8g2_font_5x7_tf);
        u8g2.setCursor(0, 8);

        if (isBatteryPower) {
            u8g2.print(F("BAT: "));
            u8g2.print(trueBatteryVoltage, 1);
            u8g2.print(F("V ["));
            u8g2.print((int)batteryPercentage);
            u8g2.print(F("%]"));
        } else {
            u8g2.print(F("SYS: SMPS NETWORK"));
        }

        u8g2.setCursor(98, 8);
        if (manualPIDMode) u8g2.print(F("[MAN]"));
        else               u8g2.print(F("[AUT]"));

        u8g2.drawHLine(0, 11, 128);

        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.setCursor(2, 24); u8g2.print(F("PV:"));
        u8g2.setFont(u8g2_font_7x14_tf);
        u8g2.setCursor(22, 25); u8g2.print(inputTemp, 1); u8g2.print(F("\xb0"));

        u8g2.setFont(u8g2_font_6x10_tf);
        u8g2.setCursor(2, 42); u8g2.print(F("SV:"));
        u8g2.setFont(u8g2_font_7x14_tf);
        u8g2.setCursor(22, 43);

        if (currentMode == MODE_SET_TEMP) u8g2.print(F("["));
        u8g2.print(setTemp, 0);
        if (currentMode == MODE_SET_TEMP) u8g2.print(F("]"));
        u8g2.print(F("\xb0"));

        u8g2.drawVLine(74, 11, 38);
        u8g2.drawHLine(0, 49, 128);

        u8g2.setFont(u8g2_font_5x7_tf);

        u8g2.setCursor(78, 22);
        if (currentMode == MODE_EDIT_KP) u8g2.print(F("[P:")); else u8g2.print(F(" P:"));
        u8g2.print(Kp, 1);
        if (currentMode == MODE_EDIT_KP) u8g2.print(F("]"));

        u8g2.setCursor(78, 32);
        if (currentMode == MODE_EDIT_KI) u8g2.print(F("[I:")); else u8g2.print(F(" I:"));
        u8g2.print(Ki, 3);
        if (currentMode == MODE_EDIT_KI) u8g2.print(F("]"));

        u8g2.setCursor(78, 42);
        if (currentMode == MODE_EDIT_KD) u8g2.print(F("[D:")); else u8g2.print(F(" D:"));
        u8g2.print(Kd, 1);
        if (currentMode == MODE_EDIT_KD) u8g2.print(F("]"));

        u8g2.setCursor(2, 59);
        if (currentMode == MODE_RUNNING) {
            u8g2.print(F("E:")); u8g2.print(tempError, 1);
            u8g2.print(F(" PWM:")); u8g2.print((int)outputPWM);

            u8g2.setCursor(94, 59);
            if (outputPWM > 180)     u8g2.print(F("~ ~ ~"));
            else if (outputPWM > 40) u8g2.print(F(" ~ ~ "));
            else if (outputPWM > 5)  u8g2.print(F("  ~  "));
            else                     u8g2.print(F("     "));
        } else {
            u8g2.print(F("B2(3s):Cycle | B1(3s):SAVE"));
        }

    } while (u8g2.nextPage());
}