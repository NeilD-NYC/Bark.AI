# Bark.AI — Autonomous Property Surveillance Rover

> A DIY autonomous ground rover built for $400 from donor consumer 
> electronics, functioning as a 24/7 AI-powered digital guard dog.

![Status](https://img.shields.io/badge/status-active%20development-brightgreen)
![Platform](https://img.shields.io/badge/platform-ESP32%20%2B%20RPi5-blue)
![AI](https://img.shields.io/badge/AI-Hailo%20NPU-orange)

## What It Does
Patrols a residential property perimeter autonomously, detects 
intruders via AI vision, and executes a deterrent response -- 
all on a 24-hour battery cycle without human intervention.

![Bark.AI parts layout](docs/build_photo.jpg)
![Architecture schematic](docs/architecture.svg)

## Architecture

[Hailo NPU] --> [Raspberry Pi 5] --> [ESP32] --> [2x BLDC Hub Motors]
Vision AI       Mission Brain      Motor Ctrl     4WD Drivetrain

## Hardware Stack
| Component | Source | Cost |
|---|---|---|
| Chassis + steering | Razor Crazy Cart (donor) | $0 |
| Drive motors | Hoverboard hub motors (donor) | $0 |
| Compute brain | Raspberry Pi 5 | ~$80 |
| Motor controller | ESP32 + custom firmware | ~$15 |
| AI accelerator | Hailo-8 NPU | ~$70 |
| Battery | 36V 20Ah Li-ion | ~$165 |
| Camera | Razer Kiyo (donor) | $0 |
| **Total** | | **~$400** |

## Software Stack
- **ESP32 firmware** (C++): BLDC ESC control, state machine, 
  telemetry, WiFi API
- **RPi5 brain** (Python): mission planning, waypoint navigation, 
  serial command dispatch
- **Hailo NPU**: real-time person/vehicle detection at the edge, 
  no cloud dependency

## Rover State Machine
`IDLE` → `PATROL` → `INVESTIGATE` → `ALERT` → `DETER` → `PATROL`

Battery watchdog triggers `RETURN_HOME` at 25%, `EMERGENCY_STOP` at 10%.

## Build Status
- [x] Chassis fabrication
- [x] ESP32 motor controller firmware
- [ ] RPi5 patrol navigation software
- [ ] Hailo NPU vision integration
- [ ] Gen 2: autonomous charging dock

## Why I Built This
Wanted a real-world platform to learn edge AI, embedded systems 
and autonomous robotics without spending $5,000 on a research robot.
