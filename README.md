# Adaptive Climate Control System (Arduino Uno, PID + Auto-Tune)

A closed-loop temperature and humidity control system built on an
Arduino Uno Rev3. It combines a **PID temperature controller** (with
derivative filtering and anti-windup), a **relay-feedback auto-tuner**
(Åström–Hägglund method) that identifies the plant and computes new
PID gains automatically, and a **hysteresis humidity controller** —
giving the project two distinct, deliberately-contrasted control
strategies to discuss in a report or defense.

Verified: this sketch was compiled end-to-end against the real
Arduino AVR core (1.8.6), the Adafruit DHT and Unified Sensor
libraries, and the LiquidCrystal_I2C library, using `avr-gcc`/`avr-g++`
directly (no simulation). It linked with **zero errors** and fits in
**59% of flash (19,160 / 32,256 B)** and **37% of SRAM (765 / 2,048
B)** on an ATmega328P. What compiling cannot catch — wiring mistakes,
a mis-addressed I2C LCD, sensor timing quirks on real hardware — still
needs a bench test with the actual parts; do that before you trust it
unattended.

## 1. Why this is a control-engineering project, not just a sensor logger

| Concept | Where it appears |
|---|---|
| Feedback control | PID loop regulating temperature via PWM heater |
| Actuator saturation & anti-windup | Clamped back-calculation in `computePID()` |
| Derivative kick avoidance | Derivative computed on measurement, not on error |
| Noise handling | Low-pass filter on the derivative term |
| System identification | Relay-feedback auto-tune estimates ultimate gain `Ku` and ultimate period `Pu` |
| Controller design rule | Ziegler–Nichols "no-overshoot" tuning from `Ku`, `Pu` |
| Alternate control strategy | Hysteresis (bang-bang) control for humidity, for comparison against PID |
| Persistence / embedded systems | EEPROM storage of gains and setpoints across power cycles |
| Fault tolerance | Sensor-timeout detection with safe-state shutdown and alarm |

This mix is what makes it "big enough" for a bachelor's final project:
you're not just reading a sensor, you're identifying a plant,
designing a controller for it, handling its non-idealities (windup,
noise, actuator limits), and validating it with data.

## 2. Hardware

| Component | Arduino Uno Pin | Notes |
|---|---|---|
| DHT22 (AM2302) data | D2 | Add a 10 kΩ pull-up between DATA and VCC if your breakout doesn't have one |
| Heater (via MOSFET/SSR) | D9 (PWM) | Never drive a resistive heater straight from an Uno pin — use a logic-level MOSFET or solid-state relay rated for your heater's current |
| Cooling fan | D10 (PWM) | Small 5V/12V fan through a transistor/MOSFET |
| Humidifier relay | D6 | Active-LOW relay module |
| Dehumidifier / exhaust fan relay | D7 | Active-LOW relay module |
| Status LED | D13 (onboard) | Solid = OK, slow blink = auto-tuning, fast blink = sensor fault |
| Buzzer | D8 | Sounds on sensor fault |
| 16x2 I2C LCD | A4 (SDA), A5 (SCL) | Default address `0x27` — some backpacks use `0x3F`; run an I2C scanner sketch if the display stays blank |

**Power note:** heaters, fans, and relays should be powered from an
appropriately-rated external supply, not the Uno's 5V regulator.
Share ground between the Uno and the external supply.

## 3. Repository layout

```
ClimateControlPID/
  ClimateControlPID.ino   # main sketch — upload this
tools/
  serial_logger.py        # logs the CSV stream to a file and live-plots it
docs/
  wiring.md               # wiring notes / breadboard guidance
.github/workflows/
  compile.yml             # CI: verifies the sketch still compiles on every push
LICENSE
README.md
```

## 4. Building / uploading

1. Install the [Arduino IDE](https://www.arduino.cc/en/software) (or
   `arduino-cli`).
2. Install libraries via Library Manager: **DHT sensor library**
   (Adafruit), **Adafruit Unified Sensor**, **LiquidCrystal_I2C**
   (by John Rickman / Frank de Brabander build — either fork works).
3. Open `ClimateControlPID/ClimateControlPID.ino`, select **Board:
   Arduino Uno**, select the correct port, and click Upload.
4. Open the Serial Monitor at **9600 baud**. Type `HELP` for the
   command list.

## 5. Serial command console

```
STATUS              show current readings and outputs
SET T <val>         set temperature setpoint (°C)
SET H <val>         set humidity setpoint (%RH)
SET KP <val>        manually set proportional gain
SET KI <val>        manually set integral gain
SET KD <val>        manually set derivative gain
AUTOTUNE            run the relay-feedback auto-tuner
LOG ON / LOG OFF    toggle CSV telemetry on the serial port
SAVE                persist current setpoints/gains to EEPROM
```

Telemetry (when logging is on) is CSV, one line every 2 s:

```
millis,tempC,humRH,setT,setH,Kp,Ki,Kd,heaterPWM,fan,humidifier,dehumid,fault
```

## 6. Running the auto-tuner

1. Set your target with `SET T 25` (or whatever setpoint you want the
   final PID to hold).
2. Type `AUTOTUNE`. The heater switches between 0 and a fixed PWM
   level (relay feedback) until the temperature settles into a
   sustained oscillation around the setpoint (needs ~6 cycles — for a
   typical small enclosure this can take several minutes to an hour,
   since it's thermal).
3. When it finishes, it prints the estimated ultimate gain/period and
   the new `Kp, Ki, Kd`, and saves them to EEPROM automatically.
4. From then on, normal PID control resumes with the new gains. You
   can always override with `SET KP/KI/KD` if you want to compare
   against manual tuning in your report — that comparison (auto-tuned
   vs. hand-tuned step response) is good material for a final project.

## 7. Logging and plotting from a PC

`tools/serial_logger.py` reads the CSV stream, saves it to a
timestamped `.csv` file, and live-plots temperature vs. setpoint and
the heater PWM — enough to produce a step-response plot for your
report. See the script's header for usage.

## 8. Suggested report content (if this is going in a thesis/portfolio)

- Block diagram of the closed loop (plant, sensor, controller, actuator)
- Open-loop step response of the enclosure (heater at fixed PWM,
  record temperature vs. time) to estimate a first-order-plus-dead-time
  model
- Auto-tune results (Ku, Pu) and the resulting gains
- Closed-loop step response with the tuned PID: rise time, overshoot,
  settling time
- A run with intentionally poor gains (e.g., `SET KI` too high) to
  show windup, then the same run with anti-windup enabled vs.
  disabled (comment out the anti-windup block to compare) — this is a
  classic, easy-to-explain demonstration for a defense.

## 9. Known limitations / things to bench-test yourself

- I compiled this against the real toolchain and libraries and it
  links cleanly, but I have no physical board, sensor, or actuators to
  run it on — validate wiring, the LCD I2C address, and actual sensor
  timing on your bench before trusting it unattended.
- The relay auto-tuner assumes the plant is slow enough that DHT22's
  ~0.5 Hz read rate doesn't alias the oscillation; for a large,
  slow-thermal enclosure this is fine, but for a fast/small setup you
  may need a faster sensor (e.g., a thermistor + ADC) for tuning to
  converge cleanly.
- `AT_CYCLES_NEEDED = 6` and `AT_RELAY_AMPLITUDE = 80` are reasonable
  defaults but plant-dependent — tune them for your enclosure size and
  heater power.

## License

MIT — see `LICENSE`.
