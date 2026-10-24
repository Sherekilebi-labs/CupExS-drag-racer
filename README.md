# CupExS Drag Racer

ESP32 firmware for a browser-controlled drag car and an independent two-lane infrared finish timer. A companion motor-dyno experiment is included separately.

## Car controller

`src/car controller.cpp` provides browser Start / Stop commands, 20 kHz 10-bit PWM, and a 60-second maximum forward run. GPIO19 is forward and GPIO21 is reverse. Both outputs are cut on stop. Build with `pio run -e car_controller` (also the default environment).

## Finish timer

`src/startfinish controller.cpp` runs on another ESP32. It alternates 36 kHz IR bursts across two lanes, provides a countdown, records each beam-break finish independently, and serves the results in a local web interface. Build with `pio run -e startfinish_controller`.

| Lane | Receiver | Transmitter |
| --- | --- | --- |
| 1 | GPIO13 | GPIO4 |
| 2 | GPIO21 | GPIO33 |

The browser's car-start command and the timer countdown are separate; this code does not implement synchronized remote launch. Change the timer's demonstration AP password before use. The prior development variant is retained under `archive/`, outside the build source directory.

## Companion dyno

`dyno/` contains an ESP32 current-limited motor-control and inertia-dyno experiment using ACS712 current sensing and MT6701 encoder feedback. Build separately with `pio run -d dyno`. Torque and power are estimates derived from a configured inertia, not certified motor measurements.

**Wiring issue to resolve before running:** the current dyno source assigns GPIO21 to both reverse motor drive and MT6701 SDA. Older wiring notes also disagree with the current motor-pin constants. Those source assignments are preserved for review, not silently changed to an unverified wiring layout. A successful software build does not resolve that physical conflict.
