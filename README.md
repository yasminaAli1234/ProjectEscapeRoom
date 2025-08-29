#  ESP32 Escape Room Game

**Author:** team 7 
**Platform:** ESP32  
**Date:** 2025  

---

## Overview

This is a multi-level interactive escape room game powered by an ESP32.  
The game integrates sensors, servos, and MQTT communication for remote monitoring and control.  

Players progress through 5 main levels:  

1. **Keypad Code** – Enter the correct 4-digit code.  
2. **LDR Sensor** – Manipulate light intensity to match a sequence.  
3. **Gas Puzzle** – Detect gas changes and adjust a potentiometer to match the target.  
4. **IR Red Light Game** – Move only during green light and avoid strikes.  
5. **Flame Sensor Code** – Detect flame and enter the displayed code.

---

##  Hardware Required

- ESP32 Dev Board  
- Keypad 3x3  
- Servo motors (Gate & Flame)  
- LDR sensor  
- MQ2 Gas Sensor  
- IR sensor  
- Flame sensor  
- LEDs (Red, Yellow, Green)  
- Buzzer  
- Buttons, resistors, and connecting wires  

---

## Software & Libraries

- **Arduino IDE**  
- Libraries:  
  - `Keypad`  
  - `ESP32Servo`  
  - `Wire`  
  - `WiFi`  
  - `WiFiClientSecure`  
  - `PubSubClient`  
  - `ArduinoJson`  

---

##  Wiring / Pin Configuration

| Component        | ESP32 Pin |
|-----------------|-----------|
| Gate Servo       | 4         |
| Flame Servo      | 13        |
| LDR              | 33        |
| Gas Sensor       | 35        |
| Potentiometer    | 34        |
| Button           | 19        |
| IR Sensor        | 32        |
| Flame Sensor     | 34        |
| Red LED          | 15        |
| Yellow LED       | 2         |
| Green LED        | 23        |
| Buzzer           | 18        |
| Keypad Rows      | 13,12,14  |
| Keypad Columns   | 25,26,27  |

---

## Network & MQTT Configuration

- WiFi: `SSID` and `Password` in code  
- MQTT Broker: HiveMQ  
  - `mqtt_server`, `mqtt_port`, `mqtt_username`, `mqtt_password`  
- Publish topic: `yasmin/sensors`  
- Subscribe topic: `game/control`  

Commands you can send remotely via MQTT:  
- `reset` – Reset the game  
- `skip` – Skip current level  
- `led_on` / `led_off` – Control Red LED  
- `servo_open` / `servo_close` – Open/close gate  

---

## Gameplay

1. **Keypad Level** – Enter the correct code to open the gate.  
2. **LDR Level** – Cover/uncover LDR to match light sequence.  
3. **Gas Puzzle** – Adjust potentiometer to match the target gas value.  
4. **IR Game** – Move only on green light; avoid moving on red light.  
5. **Flame Level** – Detect flame and type the code displayed to unlock the final servo.  

---

## Notes

- Ensure all sensors and servos are correctly connected.  
- Adjust thresholds (LDR, Gas sensor) based on your environment.  
- Use secure MQTT credentials and WiFi.  

---

## Credits

- Developed by **Yasmin Ali**  
- Inspired by IoT-based interactive escape rooms  

---

