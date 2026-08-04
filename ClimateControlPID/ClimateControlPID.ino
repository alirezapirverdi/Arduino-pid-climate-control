/*
  =====================================================================
  Adaptive Climate Control System — Arduino Uno Rev3
  =====================================================================
  A closed-loop temperature control system with PID feedback control,
  relay-based automatic PID tuning (Astrom-Hagglund method), hysteresis
  humidity control, anti-windup, sensor-fault detection, EEPROM
  parameter persistence, and a serial command console for tuning,
  logging, and live plotting.

  Hardware:
    - Arduino Uno Rev3
    - DHT22 (AM2302) temperature/humidity sensor -> D2
    - Heater actuator (MOSFET-driven resistive heater or heat pad),
      PWM output -> D9
    - Cooling fan (PWM, for active cooling / heat dissipation) -> D10
    - Humidifier relay (active-low relay module) -> D6
    - Dehumidifier / exhaust fan relay -> D7
    - Status LED -> D13 (onboard)
    - Alarm buzzer -> D8
    - 16x2 I2C LCD (PCF8574 backpack, addr 0x27) -> A4 (SDA), A5 (SCL)

  Control architecture:
    - Temperature: PID controller driving a PWM heater (0-255) with a
      secondary bang-bang cooling fan for large positive errors
      (measured temp above setpoint). Anti-windup via clamped
      back-calculation. Derivative term is filtered (low-pass) and
      computed on measurement, not on error, to avoid derivative kick.
    - Humidity: hysteresis (bang-bang) controller — simpler and more
      appropriate for slow, non-linear humidity dynamics, and it lets
      the project demonstrate two different control strategies.
    - Auto-tune: relay feedback test estimates the ultimate gain (Ku)
      and ultimate period (Pu) of the plant, then applies
      Ziegler-Nichols "no overshoot" PID rules.

  Author: (project template — customize the header before submitting)
  =====================================================================
*/

#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// ---------------------------------------------------------------
// Pin assignments
// ---------------------------------------------------------------
#define DHTPIN        2
#define DHTTYPE       DHT22
#define HEATER_PIN    9      // PWM
#define FAN_PIN       10     // PWM
#define HUMIDIFY_PIN  6      // digital relay, active LOW
#define DEHUMID_PIN   7      // digital relay, active LOW
#define STATUS_LED    13
#define BUZZER_PIN    8

// ---------------------------------------------------------------
// Timing constants
// ---------------------------------------------------------------
const unsigned long SAMPLE_TIME_MS   = 2000;  // DHT22 max ~0.5 Hz
const unsigned long LCD_UPDATE_MS    = 1000;
const unsigned long SERIAL_LOG_MS    = 2000;
const unsigned long SENSOR_TIMEOUT_MS = 8000; // consecutive failed reads -> fault

// ---------------------------------------------------------------
// EEPROM layout
// ---------------------------------------------------------------
const int EE_MAGIC_ADDR   = 0;   // 1 byte  - validity marker
const int EE_KP_ADDR      = 2;   // float
const int EE_KI_ADDR      = 6;   // float
const int EE_KD_ADDR      = 10;  // float
const int EE_TSET_ADDR    = 14;  // float
const int EE_HSET_ADDR    = 18;  // float
const int EE_MAGIC_VALUE  = 0xA5;

// ---------------------------------------------------------------
// Globals
// ---------------------------------------------------------------
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Process variables
float temperature = NAN;
float humidity     = NAN;
bool  sensorFault   = false;
unsigned long lastGoodReadMs = 0;

// Setpoints
float tempSetpoint = 25.0;   // deg C
float humSetpoint  = 50.0;   // % RH
const float HUM_HYSTERESIS = 3.0; // +/- band around setpoint

// PID gains (defaults; overwritten by EEPROM if valid, or by auto-tune)
float Kp = 18.0;
float Ki = 0.6;
float Kd = 25.0;

// PID internal state
float integral      = 0.0;
float lastMeasurement = NAN;
float lastFilteredDeriv = 0.0;
const float DERIV_FILTER_ALPHA = 0.2; // low-pass factor on derivative term
const float OUT_MIN = 0.0;
const float OUT_MAX = 255.0;
const float INTEGRAL_MAX = 255.0 / 0.6; // generous clamp, refined by anti-windup below

unsigned long lastSampleMs = 0;
unsigned long lastLcdMs    = 0;
unsigned long lastLogMs    = 0;

float heaterOutput = 0.0;
bool  fanOn         = false;
bool  humidifierOn  = false;
bool  dehumidOn     = false;

// Data logging toggle (CSV to Serial)
bool loggingEnabled = true;

// ---------------------------------------------------------------
// Auto-tune (relay feedback / Astrom-Hagglund) state machine
// ---------------------------------------------------------------
enum AutoTuneState { AT_IDLE, AT_RUNNING, AT_DONE };
AutoTuneState atState = AT_IDLE;

const float AT_RELAY_AMPLITUDE = 80.0;   // heater PWM swing (0..255 scale)
const float AT_HYSTERESIS      = 0.3;    // deg C, avoids chattering on noise
bool   atRelayHigh        = true;
unsigned long atLastSwitchMs = 0;
unsigned long atPeriodStartMs = 0;
float  atPeakMax = -1000, atPeakMin = 1000;
int    atCycleCount = 0;
const int AT_CYCLES_NEEDED = 6;
float  atPeriodSum = 0;
float  atAmplitudeSum = 0;
int    atPeriodSamples = 0;

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(9600);
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  pinMode(HUMIDIFY_PIN, OUTPUT);
  pinMode(DEHUMID_PIN, OUTPUT);
  pinMode(STATUS_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Relays are active-LOW: HIGH = off, ensure safe startup state
  digitalWrite(HUMIDIFY_PIN, HIGH);
  digitalWrite(DEHUMID_PIN, HIGH);
  analogWrite(HEATER_PIN, 0);
  analogWrite(FAN_PIN, 0);
  digitalWrite(BUZZER_PIN, LOW);

  dht.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Climate Ctrl PID");
  lcd.setCursor(0, 1);
  lcd.print("Booting...");

  loadSettingsFromEEPROM();

  Serial.println(F("========================================"));
  Serial.println(F(" Adaptive Climate Control System - Ready"));
  Serial.println(F(" Type HELP for the command list"));
  Serial.println(F("========================================"));
  printStatusHeader();

  delay(1500);
  lcd.clear();
}

// =====================================================================
// MAIN LOOP
// =====================================================================
void loop() {
  handleSerialCommands();

  unsigned long now = millis();

  if (now - lastSampleMs >= SAMPLE_TIME_MS) {
    lastSampleMs = now;
    readSensors();

    if (!sensorFault) {
      if (atState == AT_RUNNING) {
        runAutoTuneStep(now);
      } else {
        float dt = SAMPLE_TIME_MS / 1000.0;
        heaterOutput = computePID(tempSetpoint, temperature, dt);
        applyTemperatureControl(heaterOutput, temperature, tempSetpoint);
      }
      applyHumidityControl(humidity, humSetpoint);
    } else {
      safeShutdownOutputs();
    }

    if (loggingEnabled && (now - lastLogMs >= SERIAL_LOG_MS)) {
      lastLogMs = now;
      logStatusCSV();
    }
  }

  if (now - lastLcdMs >= LCD_UPDATE_MS) {
    lastLcdMs = now;
    updateLCD();
  }

  updateStatusLED(now);
  handleAlarms();
}

// =====================================================================
// SENSOR HANDLING
// =====================================================================
void readSensors() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t)) {
    // Keep last good value but flag fault if timeout exceeded
    if (millis() - lastGoodReadMs > SENSOR_TIMEOUT_MS && lastGoodReadMs != 0) {
      sensorFault = true;
    } else if (lastGoodReadMs == 0) {
      // no good reading has ever happened yet; treat as fault after timeout too
      sensorFault = (millis() > SENSOR_TIMEOUT_MS);
    }
    return;
  }

  // Basic plausibility check for a DHT22 (reject clearly bad packets)
  if (t < -40 || t > 80 || h < 0 || h > 100) {
    return;
  }

  temperature = t;
  humidity = h;
  lastGoodReadMs = millis();
  sensorFault = false;
}

// =====================================================================
// PID CONTROLLER (temperature)
//   - Derivative on measurement (not on error) to avoid derivative kick
//   - Low-pass filtered derivative term to reduce sensor-noise sensitivity
//   - Clamped back-calculation anti-windup
// =====================================================================
float computePID(float setpoint, float measurement, float dt) {
  float error = setpoint - measurement;

  // --- Proportional ---
  float pTerm = Kp * error;

  // --- Integral (accumulate first, clamp applied after computing output) ---
  integral += Ki * error * dt;

  // --- Derivative on measurement, filtered ---
  float rawDeriv = 0.0;
  if (!isnan(lastMeasurement) && dt > 0.0) {
    rawDeriv = -(measurement - lastMeasurement) / dt; // negative: derivative of error = -d(measurement)/dt
  }
  lastFilteredDeriv = DERIV_FILTER_ALPHA * rawDeriv + (1.0 - DERIV_FILTER_ALPHA) * lastFilteredDeriv;
  float dTerm = Kd * lastFilteredDeriv;
  lastMeasurement = measurement;

  // --- Unclamped output ---
  float output = pTerm + integral + dTerm;

  // --- Anti-windup: clamp output, and back-off the integral term if saturated ---
  float clampedOutput = output;
  if (clampedOutput > OUT_MAX) {
    clampedOutput = OUT_MAX;
    if (error > 0) integral -= Ki * error * dt; // undo integration that caused windup
  } else if (clampedOutput < OUT_MIN) {
    clampedOutput = OUT_MIN;
    if (error < 0) integral -= Ki * error * dt;
  }

  return clampedOutput;
}

// =====================================================================
// ACTUATION
// =====================================================================
void applyTemperatureControl(float pidOutput, float measTemp, float setpoint) {
  // Heater gets the PID output directly (0-255 PWM)
  analogWrite(HEATER_PIN, (int)pidOutput);

  // Cooling fan: simple bang-bang assist when well above setpoint,
  // since the heater alone cannot cool the enclosure.
  float overshoot = measTemp - setpoint;
  if (overshoot > 1.0) {
    fanOn = true;
    int fanPWM = (int)constrain((overshoot - 1.0) * 80.0, 60, 255);
    analogWrite(FAN_PIN, fanPWM);
  } else if (overshoot < 0.3) {
    fanOn = false;
    analogWrite(FAN_PIN, 0);
  }
}

void applyHumidityControl(float measHum, float setpoint) {
  if (isnan(measHum)) return;

  float lower = setpoint - HUM_HYSTERESIS;
  float upper = setpoint + HUM_HYSTERESIS;

  if (measHum < lower) {
    humidifierOn = true;
    dehumidOn = false;
  } else if (measHum > upper) {
    humidifierOn = false;
    dehumidOn = true;
  } else if (measHum >= setpoint - 0.5 && measHum <= setpoint + 0.5) {
    // dead-band: both off once close enough to setpoint
    humidifierOn = false;
    dehumidOn = false;
  }
  // else: keep the previous state (true hysteresis behavior)

  digitalWrite(HUMIDIFY_PIN, humidifierOn ? LOW : HIGH);  // active LOW relay
  digitalWrite(DEHUMID_PIN, dehumidOn ? LOW : HIGH);
}

void safeShutdownOutputs() {
  analogWrite(HEATER_PIN, 0);
  analogWrite(FAN_PIN, 0);
  digitalWrite(HUMIDIFY_PIN, HIGH);
  digitalWrite(DEHUMID_PIN, HIGH);
  heaterOutput = 0;
  fanOn = false;
  humidifierOn = false;
  dehumidOn = false;
}

// =====================================================================
// RELAY-FEEDBACK AUTO-TUNE (Astrom-Hagglund)
//   Drives the heater as a two-level relay around the setpoint and
//   measures the resulting sustained oscillation. From the oscillation
//   period (Pu) and amplitude (a) it estimates the ultimate gain
//   Ku = 4*d / (pi*a), then applies Ziegler-Nichols "no overshoot"
//   (Pessen-like conservative) PID rules.
// =====================================================================
void startAutoTune() {
  atState = AT_RUNNING;
  atRelayHigh = true;
  atCycleCount = 0;
  atPeakMax = -1000;
  atPeakMin = 1000;
  atPeriodSum = 0;
  atAmplitudeSum = 0;
  atPeriodSamples = 0;
  atLastSwitchMs = millis();
  atPeriodStartMs = millis();
  integral = 0; // reset PID state so it doesn't interfere afterward
  Serial.println(F(">> Auto-tune started. Do not change setpoint until it completes."));
}

void runAutoTuneStep(unsigned long now) {
  float error = tempSetpoint - temperature;

  // Track peaks for amplitude estimation
  if (temperature > atPeakMax) atPeakMax = temperature;
  if (temperature < atPeakMin) atPeakMin = temperature;

  // Relay logic with hysteresis to avoid chatter from sensor noise
  if (atRelayHigh && error < -AT_HYSTERESIS) {
    atRelayHigh = false;
    onAutoTuneSwitch(now);
  } else if (!atRelayHigh && error > AT_HYSTERESIS) {
    atRelayHigh = true;
    onAutoTuneSwitch(now);
  }

  float out = atRelayHigh ? AT_RELAY_AMPLITUDE : 0.0;
  analogWrite(HEATER_PIN, (int)out);
  analogWrite(FAN_PIN, 0);
  heaterOutput = out;

  if (atCycleCount >= AT_CYCLES_NEEDED) {
    finishAutoTune();
  }
}

void onAutoTuneSwitch(unsigned long now) {
  // A full cycle = two switches (high->low->high). Count on the
  // low->high transition, once we've discarded the first noisy cycle.
  if (atRelayHigh) {
    unsigned long period = now - atPeriodStartMs;
    atPeriodStartMs = now;
    atCycleCount++;

    if (atCycleCount > 1) { // discard first cycle (transient)
      atPeriodSum += period;
      atAmplitudeSum += (atPeakMax - atPeakMin);
      atPeriodSamples++;
    }
    atPeakMax = -1000;
    atPeakMin = 1000;
  }
}

void finishAutoTune() {
  analogWrite(HEATER_PIN, 0);
  atState = AT_DONE;

  if (atPeriodSamples < 1) {
    Serial.println(F(">> Auto-tune FAILED: not enough valid oscillation cycles."));
    atState = AT_IDLE;
    return;
  }

  float Pu = (atPeriodSum / atPeriodSamples) / 1000.0; // seconds
  float a  = (atAmplitudeSum / atPeriodSamples) / 2.0; // oscillation half-amplitude
  float d  = AT_RELAY_AMPLITUDE / 2.0;                  // relay half-amplitude (PWM units)

  if (a <= 0.001) {
    Serial.println(F(">> Auto-tune FAILED: measured amplitude too small."));
    atState = AT_IDLE;
    return;
  }

  float Ku = (4.0 * d) / (3.14159265 * a);

  // Ziegler-Nichols "no overshoot" PID rule set (more conservative than
  // classic ZN — appropriate for a thermal plant with actuator limits).
  Kp = 0.2 * Ku;
  Ki = 0.4 * Ku / Pu;      // Ki = Kp / Ti,  Ti = Pu/2
  Kd = 0.066 * Ku * Pu;    // Kd = Kp * Td,  Td = Pu/3

  integral = 0;
  saveSettingsToEEPROM();

  Serial.println(F(">> Auto-tune COMPLETE"));
  Serial.print(F("   Ultimate gain Ku=")); Serial.println(Ku, 3);
  Serial.print(F("   Ultimate period Pu=")); Serial.print(Pu, 2); Serial.println(F(" s"));
  Serial.print(F("   New gains -> Kp=")); Serial.print(Kp, 3);
  Serial.print(F("  Ki=")); Serial.print(Ki, 4);
  Serial.print(F("  Kd=")); Serial.println(Kd, 3);

  atState = AT_IDLE;
}

// =====================================================================
// SERIAL COMMAND CONSOLE
//   Commands (case-insensitive), newline-terminated:
//     HELP
//     STATUS
//     SET T <value>        set temperature setpoint (C)
//     SET H <value>        set humidity setpoint (%RH)
//     SET KP <value> / SET KI <value> / SET KD <value>
//     AUTOTUNE              start relay-feedback auto-tune
//     LOG ON / LOG OFF       toggle CSV serial logging
//     SAVE                   persist current setpoints/gains to EEPROM
// =====================================================================
String serialBuffer = "";

void handleSerialCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        processCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 40) serialBuffer = ""; // guard against garbage
    }
  }
}

void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();

  if (cmd == "HELP") {
    printHelp();
  } else if (cmd == "STATUS") {
    printStatusHeader();
    logStatusCSV();
  } else if (cmd.startsWith("SET T ")) {
    tempSetpoint = cmd.substring(6).toFloat();
    Serial.print(F("Temperature setpoint = ")); Serial.println(tempSetpoint);
  } else if (cmd.startsWith("SET H ")) {
    humSetpoint = cmd.substring(6).toFloat();
    Serial.print(F("Humidity setpoint = ")); Serial.println(humSetpoint);
  } else if (cmd.startsWith("SET KP ")) {
    Kp = cmd.substring(7).toFloat();
    Serial.print(F("Kp = ")); Serial.println(Kp, 4);
  } else if (cmd.startsWith("SET KI ")) {
    Ki = cmd.substring(7).toFloat();
    Serial.print(F("Ki = ")); Serial.println(Ki, 4);
  } else if (cmd.startsWith("SET KD ")) {
    Kd = cmd.substring(7).toFloat();
    Serial.print(F("Kd = ")); Serial.println(Kd, 4);
  } else if (cmd == "AUTOTUNE") {
    startAutoTune();
  } else if (cmd == "LOG ON") {
    loggingEnabled = true;
    Serial.println(F("Logging ON"));
  } else if (cmd == "LOG OFF") {
    loggingEnabled = false;
    Serial.println(F("Logging OFF"));
  } else if (cmd == "SAVE") {
    saveSettingsToEEPROM();
    Serial.println(F("Settings saved to EEPROM."));
  } else {
    Serial.println(F("Unknown command. Type HELP."));
  }
}

void printHelp() {
  Serial.println(F("--- Commands ---"));
  Serial.println(F("STATUS            show current readings and outputs"));
  Serial.println(F("SET T <val>       set temperature setpoint (C)"));
  Serial.println(F("SET H <val>       set humidity setpoint (%RH)"));
  Serial.println(F("SET KP/KI/KD <v>  manually set PID gains"));
  Serial.println(F("AUTOTUNE          run relay-feedback auto-tune"));
  Serial.println(F("LOG ON / LOG OFF  toggle CSV serial logging"));
  Serial.println(F("SAVE              persist setpoints/gains to EEPROM"));
}

void printStatusHeader() {
  Serial.println(F("millis,tempC,humRH,setT,setH,Kp,Ki,Kd,heaterPWM,fan,humidifier,dehumid,fault"));
}

void logStatusCSV() {
  Serial.print(millis());        Serial.print(",");
  Serial.print(temperature, 2);  Serial.print(",");
  Serial.print(humidity, 2);     Serial.print(",");
  Serial.print(tempSetpoint, 2); Serial.print(",");
  Serial.print(humSetpoint, 2);  Serial.print(",");
  Serial.print(Kp, 3);           Serial.print(",");
  Serial.print(Ki, 4);           Serial.print(",");
  Serial.print(Kd, 3);           Serial.print(",");
  Serial.print((int)heaterOutput); Serial.print(",");
  Serial.print(fanOn ? 1 : 0);      Serial.print(",");
  Serial.print(humidifierOn ? 1 : 0); Serial.print(",");
  Serial.print(dehumidOn ? 1 : 0);    Serial.print(",");
  Serial.println(sensorFault ? 1 : 0);
}

// =====================================================================
// DISPLAY
// =====================================================================
void updateLCD() {
  lcd.setCursor(0, 0);
  if (sensorFault) {
    lcd.print("SENSOR FAULT!   ");
  } else {
    lcd.print("T:");
    lcd.print(temperature, 1);
    lcd.print((char)223); // degree symbol
    lcd.print("C H:");
    lcd.print(humidity, 0);
    lcd.print("%  ");
  }

  lcd.setCursor(0, 1);
  if (atState == AT_RUNNING) {
    lcd.print("AutoTune... c");
    lcd.print(atCycleCount);
    lcd.print("    ");
  } else {
    lcd.print("SP:");
    lcd.print(tempSetpoint, 1);
    lcd.print(" PWM:");
    lcd.print((int)heaterOutput);
    lcd.print("   ");
  }
}

// =====================================================================
// STATUS LED / ALARMS
// =====================================================================
void updateStatusLED(unsigned long now) {
  if (sensorFault) {
    digitalWrite(STATUS_LED, (now / 250) % 2); // fast blink = fault
  } else if (atState == AT_RUNNING) {
    digitalWrite(STATUS_LED, (now / 600) % 2); // slow blink = auto-tuning
  } else {
    digitalWrite(STATUS_LED, HIGH); // solid = normal operation
  }
}

void handleAlarms() {
  if (sensorFault) {
    unsigned long now = millis();
    digitalWrite(BUZZER_PIN, ((now / 1000) % 2) ? HIGH : LOW);
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }
}

// =====================================================================
// EEPROM PERSISTENCE
// =====================================================================
void saveSettingsToEEPROM() {
  EEPROM.write(EE_MAGIC_ADDR, EE_MAGIC_VALUE);
  EEPROM.put(EE_KP_ADDR, Kp);
  EEPROM.put(EE_KI_ADDR, Ki);
  EEPROM.put(EE_KD_ADDR, Kd);
  EEPROM.put(EE_TSET_ADDR, tempSetpoint);
  EEPROM.put(EE_HSET_ADDR, humSetpoint);
}

void loadSettingsFromEEPROM() {
  if (EEPROM.read(EE_MAGIC_ADDR) != EE_MAGIC_VALUE) {
    Serial.println(F("No saved settings found; using defaults."));
    return;
  }
  EEPROM.get(EE_KP_ADDR, Kp);
  EEPROM.get(EE_KI_ADDR, Ki);
  EEPROM.get(EE_KD_ADDR, Kd);
  EEPROM.get(EE_TSET_ADDR, tempSetpoint);
  EEPROM.get(EE_HSET_ADDR, humSetpoint);
  Serial.println(F("Loaded settings from EEPROM."));
}
