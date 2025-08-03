# PST-F1 Bolt-On Daughter Board for ESP32-CAN-X2

A small daughter-board that lets you easily attach a Bosch PST-F1 combined pressure + temperature sensor to the AutosportLabs ESP32-CAN-X2.

## Overview

This daughter-board provides ESP32-CAN-X2 an interface for the Bosch PST-F1 combined pressure and temperature sensor (0–10 bar pressure + NTC temperature). It uses a 4-pin JST connector for sensor inputs. The board also passes through CAN2 signals from the ESP32-CAN-X2.


## Features

- **PST-F1 support**: 4-pin input (5 V, PRESS_SIG, TEMP_SIG, GND)
- **Voltage scaling & protection** on both sensor outputs
- **0.1 µF decoupling** on analog rails
- **Pass-through** of CAN2 channel


## Hardware Requirements

- [AutosportLabs ESP32-CAN-X2](https://wiki.autosportlabs.com/ESP32-CAN-X2) board
- [Bosch PST-F1 combined pressure-temperature sensor](https://www.bosch-motorsport.com/content/downloads/Raceparts/en-GB/54249355.html)


## Pinout & Connections

| PST-F1 Pin | Signal       | Daughter-Board Net | ESP32-CAN-X2 Pin |
|:----------:|:-------------|:-------------------|:----------------:|
| 1          | +5 V Supply  | 5V                 | SV1-pin 15 (5 V) |
| 2          | PRESS_SIG    | PRESS_ADC          | SV2-pin 13 (GPIO)|
| 3          | TEMP_SIG     | TEMP_ADC           | SV2-pin 12 (GPIO)|
| 4          | GND          | GND                | SV1-pin 16 (GND) |


## Analog Signal Scaling/Divider

### Pressure (0–10 bar → 0.32–2.88 V ADC)

- At 0.5 V (@ 0 bar / 0 PSI) -> ADC ≈ 0.32 V
- At 4.5 V (@ 10 bar / 145 PSI) -> ADC ≈ 2.88 V

``` mermaid
flowchart LR
  PSTF1_P["PST-F1 Pressure Out<br/>0.5 V @ 0 bar – 4.5 V @ 10 bar"]
  PRESS_ADC["PRESS_ADC Node"]
  R_PRESS_TOP["R_PRESS_TOP<br/>5.6 kΩ"]
  R_PRESS_BOT["R_PRESS_BOT<br/>10 kΩ"]
  D1["D1 BAT54<br/>(Cathode→Node, Anode→GND)"]
  C1["C1 Filter Cap<br/>0.1 µF"]
  GND["GND"]
  GPIO["GPIO 35"]

  PSTF1_P --- R_PRESS_TOP --- PRESS_ADC
  PRESS_ADC --- R_PRESS_BOT --- GND
  PRESS_ADC --- D1 --- GND
  PRESS_ADC --- C1 --- GPIO
```

### Temperature (NTC 58.1 Ω-44.864 kΩ → 0.01–2.9 V ADC)

- At cold (44 kΩ) → ADC ≈ 2.9 V
- At hot (58 Ω) → ADC ≈ 0.01 V


``` mermaid
flowchart LR
  PSTF1_P["PST-F1 Temperature Out<br/>44 kΩ @ –40 °C - 58 Ω @ +150 °C"]
  R_TEMP_TOP["R_TEMP_TOP<br/>6.8 kΩ"]
  TEMP_ADC["TEMP_ADC Node"]
  C2["C2 Filter Cap<br/>0.1 µF"]
  GND["GND"]
  VCC["3.3 V"]
  GPIO["GPIO 36"]

  PSTF1_P --- TEMP_ADC
  TEMP_ADC --- C2 --- GND
  TEMP_ADC --- R_TEMP_TOP --- VCC
  TEMP_ADC --- GPIO
```
