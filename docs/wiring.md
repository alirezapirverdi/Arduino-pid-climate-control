# Wiring Guide

## Pin map

| Signal | Arduino Uno Pin | Type |
|---|---|---|
| DHT22 DATA | D2 | Digital input, 10kΩ pull-up to VCC |
| Heater gate/control | D9 | PWM output → MOSFET/SSR gate |
| Fan control | D10 | PWM output → MOSFET/transistor base |
| Humidifier relay IN | D6 | Digital output, active LOW |
| Dehumidifier relay IN | D7 | Digital output, active LOW |
| Buzzer | D8 | Digital output |
| Status LED | D13 | Onboard LED, no wiring needed |
| LCD SDA | A4 | I2C data |
| LCD SCL | A5 | I2C clock |

## Notes

- **Never** switch a heater directly off a digital pin. Use a
  logic-level N-channel MOSFET (e.g., IRLZ44N) or a solid-state relay
  rated for the heater's current, with a flyback/snubber as
  appropriate for the load type.
- Relay modules are commonly **active LOW** (`LOW` = energized). If
  your specific module is active HIGH, swap the polarity in
  `applyHumidityControl()`.
- DHT22 needs a data pull-up resistor (often already on the breakout
  board — check before adding a second one).
- If the LCD backlight comes on but shows nothing, the I2C address is
  probably wrong. Run a standard I2C scanner sketch to find it (common
  alternates: `0x3F`).
- Power the heater/fan/relays from an external supply matched to their
  current draw; do not pull heater current through the Uno's 5V pin.
  Common ground between the Uno and the external supply is required.
