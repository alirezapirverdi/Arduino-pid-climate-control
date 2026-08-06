# Arduino PID Climate Control

An adaptive climate control system built as an independent extension of an earlier,
physically implemented greenhouse control project.

## Background

In 2018–2019, as my undergraduate final project (B.S. Electrical Engineering,
Control Systems concentration), I designed, simulated in Proteus, and **physically
implemented** a closed-loop discrete PID temperature control system on an Arduino
Uno (ATmega328P) to regulate greenhouse temperature.

This repository is a **more advanced, independent extension** of that original
project, built more recently to explore adaptive control techniques beyond the
original scope. It is **not** the original 2018–2019 codebase.

## What's in this repository

- A PID temperature controller with derivative filtering and anti-windup
- A relay-feedback auto-tuner (Astrom-Hagglund method) that identifies the plant
  and automatically computes new PID gains
- A contrasting hysteresis-based humidity controller, included for comparison
  against the PID approach
- EEPROM persistence for tuned gains across power cycles

## Status

This sketch has been compiled and verified end-to-end against the real Arduino AVR
core. **It has not been run on physical hardware** — I do not currently have a
physical board, sensor, or actuators to test it on. If you build this yourself,
validate wiring and sensor calibration on your bench before trusting it unattended.

## Relation to the original project

| | Original Greenhouse Project (2018–2019) | This Repository |
|---|---|---|
| Status | Physically built and implemented | Software-verified only |
| Hardware | Arduino Uno (ATmega328P) | Arduino Uno (ATmega328P), untested |
| Control approach | Closed-loop discrete PID | PID + auto-tuning + anti-windup |
| Scope | Temperature only | Temperature (adaptive) + humidity (hysteresis) |

## License

MIT
