# ESP32 RTOS Impact Detection System

> Real-time motion and impact detection with severity classification using FreeRTOS, I²C accelerometer, and PWM alert control.

---

## Overview

A bare-metal embedded system built on the **ESP32** and **FreeRTOS** that continuously monitors acceleration data from a **KX13X accelerometer** via I²C, classifies events through a **Finite State Machine**, and triggers a **PWM buzzer alert** on impact detection — all running across three concurrent RTOS tasks with queue-based inter-task communication.

Built as a practical exploration of real-time embedded design patterns: multi-task scheduling, sensor calibration, vector-based signal processing, and non-blocking alert systems.

---

## Features

- **3-Task FreeRTOS Architecture** — Sensor acquisition, signal processing, and alert output run as independent tasks communicating via RTOS queues
- **Register-Level I²C Driver** — Direct register access to KX13X accelerometer without abstraction libraries
- **Sensor Calibration** — 50-sample baseline averaging at startup eliminates gravity-induced offset from all readings
- **Vector Delta Motion Detection** — Computes 3-axis Euclidean delta between consecutive samples for direction-agnostic impact sensing
- **Finite State Machine (FSM)** — Three-state event classifier (IDLE → IMPACT → STABLE) with timing-based lockout to suppress post-impact bounce
- **Peak Severity Classification** — Tracks peak delta throughout the impact phase; classifies as MINOR / MODERATE / SEVERE at transition to STABLE
- **Non-Blocking PWM Alert** — LEDC-driven buzzer with independent timing logic; extends alarm window on repeated impacts

---

## Hardware

| Component | Role |
|-----------|------|
| ESP32 DevKit | Main MCU — runs FreeRTOS, I²C master, LEDC PWM |
| KX13X Accelerometer | 3-axis accelerometer via I²C |
| Passive Buzzer | PWM alert output |

### Pin Configuration

| Signal | ESP32 GPIO |
|--------|-----------|
| I²C SDA | GPIO 13 |
| I²C SCL | GPIO 14 |
| Buzzer PWM | GPIO 25 |
| I²C Address (KX13X) | `0x1E` |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        app_main()                           │
│         I2C Init → Sensor Init → Buzzer Init → Calibrate    │
│              → Create Queues → Spawn 3 Tasks                │
└─────────────────────────────────────────────────────────────┘
                              │
          ┌───────────────────┼───────────────────┐
          ▼                   ▼                   ▼
   ┌─────────────┐    ┌─────────────────┐   ┌──────────────┐
   │ sensorTask  │    │  processTask    │   │  alertTask   │
   │  Priority 3 │    │  Priority 2     │   │  Priority 1  │
   │             │    │                 │   │              │
   │ Poll KX13X  │    │ FSM + Signal    │   │ Buzzer PWM   │
   │ via I²C     │───▶│ Processing      │──▶│ Control      │
   │             │    │                 │   │              │
   │ 10ms cycle  │    │ Queue-driven    │   │ 200ms beep   │
   └─────────────┘    └─────────────────┘   └──────────────┘
        │                      │                    ▲
        ▼                      ▼                    │
   sensorQueue            alertQueue ───────────────┘
   (int16_t[3])           (int)
```

---

## Finite State Machine

```
                      delta > IMPACT_THRESHOLD
         ┌─────────────────────────────────────────┐
         │                                         │
         ▼                                         │
    ┌─────────┐   delta > THRESHOLD           ┌──────────┐
    │  IDLE   │ ─────────────────────────────▶│  IMPACT  │
    └─────────┘                               └──────────┘
         ▲                                         │
         │                                         │ lockout expires &&
         │ stable_hold >= 1000ms                   │ delta < STABLE_THRESHOLD
         │                                         ▼
         └───────────────────────────────── ┌──────────┐
                                            │  STABLE  │
                                            └──────────┘
```

| State | Entry Condition | Action |
|-------|----------------|--------|
| `IDLE` | Startup / post-stable timeout | Monitoring for impacts |
| `IMPACT` | `delta > 3500.0` | Log event, track peak delta, trigger buzzer |
| `STABLE` | Post-lockout & `delta < 300.0` | Classify severity, wait for 1s hold |

### Severity Classification (at STABLE transition)

| Peak Delta | Severity |
|-----------|----------|
| > 7000 | `SEVERE` |
| > 5000 | `MODERATE` |
| ≤ 5000 | `MINOR` |

---

## Signal Processing

Raw I²C samples are processed as follows:

```
raw_x, raw_y, raw_z          (int16_t from registers)
        │
        ▼
offset_x = raw - baseline    (gravity & sensor offset removed)
        │
        ▼
delta = √(Δx² + Δy² + Δz²)  (frame-to-frame Euclidean distance)
        │
        ▼
FSM threshold comparison
```

The **baseline calibration** (50 samples at startup) removes static gravity contribution and sensor bias, leaving only dynamic motion signal.

---

## Thresholds & Timing

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `IMPACT_THRESHOLD` | 3500.0 | Minimum delta to register an impact |
| `STABLE_THRESHOLD` | 300.0 | Maximum delta to consider motion settled |
| `STABLE_HOLD_MS` | 1000 ms | Duration motion must stay below threshold before returning to IDLE |
| `IMPACT_LOCKOUT_MS` | 300 ms | Bounce suppression window after first impact spike |
| Alert duration | 5000 ms | Buzzer active window from last impact |

---

## Getting Started

### Prerequisites

- [ESP-IDF v5.x](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)
- ESP32 DevKit
- KX13X accelerometer module
- Passive buzzer

### Build & Flash

```bash
# Clone the repo
git clone https://github.com/RevanMJ10/rtos-impact-detection-esp32
cd rtos-impact-detection-esp32

# Set target
idf.py set-target esp32

# Build
idf.py build

# Flash and monitor
idf.py -p PORT flash monitor #change PORT to your port
```

### Expected Serial Output

```
WHO_AM_I: 0x3D (expect 0x3D or 0x46)
Sensor initialized.
Calibrating... keep sensor still.
Baseline → X:142.30 Y:-88.10 Z:16380.20
Delta:12.3 | State:IDLE
Delta:18.7 | State:IDLE
>>> IMPACT #1 detected | Delta:4821.5 <<<
Delta:4821.5 | State:IMPACT
>>> IMPACT #1 | MODERATE | Peak Delta:5312.0 <<<
Delta:198.2 | State:STABLE
--- Settled → IDLE ---
```

---

## Project Structure

```
rtos-impact-detection-esp32/
├── main/
│   ├── main.c              # Full application source
│   └── CMakeLists.txt
├── docs/
│   └── circuit_diagram.png # Wiring reference
├── CMakeLists.txt
├── sdkconfig
├── .gitignore
└── README.md
```

---

## Key Design Decisions

**Why FreeRTOS queues instead of shared globals?**
Queues provide thread-safe data handoff between tasks without needing mutexes for this use case. The sensor task produces data; the process task consumes it — a clean producer-consumer pattern that avoids race conditions.

**Why a lockout period after impact?**
Mechanical impacts produce ringing — rapid oscillations immediately after the primary spike. Without a 300ms lockout, a single drop registers as 3–4 impact events. The lockout ignores all threshold checks during this window.

**Why classify severity at STABLE entry, not at IMPACT entry?**
The first sample that crosses the threshold isn't necessarily the peak — the signal often continues rising for several frames. Tracking `max_delta` throughout the IMPACT state and reading it only at transition gives an accurate representation of the event's true intensity.

---

## Tech Stack

`Embedded C` · `ESP32` · `FreeRTOS` · `I²C` · `KX13X Accelerometer` · `LEDC PWM` · `ESP-IDF`

---

## Author

**Revan MJ**
B.Tech Electronics and Computer Engineering — VIT Chennai

[LinkedIn](https://linkedin.com/in/revanmj) · [GitHub](https://github.com/RevanMJ10)
