# CAN Pulse

## Overview

A simple ESP32-based device that listens to and forwards CAN bus data from the 2022+ Toyota GR86 / Subaru BRZ (ZN8/ZD8).
The goal is to make CAN bus data available in [Racechrono](https://racechrono.com).
It taps into the ASC unit (the “GazooKazoo”), which reads CAN data to produce fake engine noise. Many owners leave it unplugged anyway, we repurpose the data stream from there.

## Hardware

### [ASL ESP-CAN-X2](https://wiki.autosportlabs.com/ESP32-CAN-X2)
### [ASL Gps-bolt-on](https://wiki.autosportlabs.com/Gps-bolt-on)

## Oil Pressure Oil Temperature Bolt-on

### Status

Abandon for now or at least deprioritised, due to lack of PCB design skills.

### Motivation

The factory setup leaves much to be desired:
- The gauge cluster on BRZ lacks a proper, high-resolution oil temperature gauge.
- Oil pressure data is absent.

This project aims to fill that gap by:
- Forwarding CAN data from the ASC (Active Sound Control) unit to a new CAN output.
- Reading engine oil pressure using a Bosch analog fluid pressure sensor (0 to 10 bar).
- Reading engine oil temperature using a Bosch analog fluid temperature sensor (-40 to 150 °C).
- Injecting that data into the new CAN bus stream so that digital dashboards and logging tools (e.g. RaceChrono) can display it.

Accurate oil pressure monitoring is critical in performance and track environments — especially given the FA24 platform’s known vulnerability to oil starvation during high-G right-hand turns.

### Features

- Dual-CAN forwarding: Reads from CAN1 (TWAI), writes to CAN2 (MCP2515).
- Sensor integration: Adds real-time oil pressure and temperature data (40Hz sampling rate, 20Hz update rate).
- Plug-and-play install: Designed to interface cleanly with the ASC connector and ESP32.

### Hardware

#### [Custom ASL ESP32-CAN-X2 Bolt-on](./bolt-on-bosch-pst-f-1/)
#### [Bosch Pressure Sensor Combined PST-F 1](https://www.bosch-motorsport.com/content/downloads/Raceparts/en-GB/54249355.html)

## License

- **Firmware** (ESP32 code): [GPLv3](./LICENSE)
- **Hardware design** (schematics, PCB layout): [CERN-OHL-S v2](./bolt-on-bosch-pst-f-1/LICENSE)
