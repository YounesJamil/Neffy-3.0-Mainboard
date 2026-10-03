# Neffy Mainboard

Firmware for the mainboard of **Neffy**, built on an ESP32-S3. The board
reads Neffy's sensors and drives its motors and LEDs, but it does not decide
anything itself: the other board (a Raspberry Pi) tells it what to do over
UART, and the mainboard reports back what it senses.

## What the board does

| Part | Behaviour |
|---|---|
| **Person sensor** (Omron D6T thermal sensor, I2C) | Reports whether someone is in front of Neffy: `1` while it is at least ~1 °C warmer than the learned room temperature (someone standing in front gives +4..6 °C, 1 m away about +2 °C), `0` otherwise. Read every 100 ms, sent on every change and repeated every 500 ms. The room temperature and the threshold adapt to the room and the sensor noise. |
| **Pockets** (two velostat pressure pads) | Report clicks, left and right. The detector calibrates itself (rest level, noise, typical click height) and also catches light clicks. Per-pocket fine-tuning: `PRESSURE_SENSITIVITY_L/R` in `config.h`. |
| **Breathing detection** | On request, a 30 s exercise: press a pocket while inhaling, let go while exhaling. The breathing rate is taken from the mean time between inhales and answered as relaxed / normal / stressed. |
| **Motors** (body = linear axis, head = rotation; TMC2225 stepper drivers) | Home against their endstops at every start, then only move on `MOVE_*` commands. Each finished move is answered with the position reached. |
| **LEDs** (WS2812B: a strip under each pocket, a ring of 12) | Follow the head: dim when it is down, full when it reaches its wake-up position, over the whole move. A click flashes its pocket strip; during breathing detection all LEDs grow brighter while inhaling. |
| **Speaker mute** | Switched by the `MUTE` command. |

## UART protocol

Every message is `FA <id> <length> <payload…>`, 115200 baud, on GPIO 43 (TX) and
44 (RX). The command list lives in the shared library
[Neffy_UART_Interface](https://github.com/LegitimateShake/Neffy_UART_Interface)
(`Supported_Commands.h`). Multi-byte numbers are sent high byte first.

**Received by the mainboard**

| ID | Command | Payload |
|---|---|---|
| `0x01` | `MUTE` | 1 byte: `01` = speaker muted, `00` = sound on (sound is on after start) |
| `0x04` | `MOVE_BODY_IN_TIME` | 4 bytes: target position (mm × 10), duration (s × 10) |
| `0x05` | `MOVE_HEAD_IN_TIME` | 4 bytes: target angle (deg × 10), duration (s × 10) |
| `0x06` | `MOVE_BODY_HEAD_IN_TIME` | 8 bytes: body position and duration, then head angle and duration |
| `0x10` | `BREATHING_DETECTION_START` | 1 byte `00`: start the 30 s breathing detection |

**Sent by the mainboard**

| ID | Message | Payload |
|---|---|---|
| `0x03` | `GET_PERSON_READING` | 1 byte: `01` = someone there, `00` = nobody |
| `0x08` / `0x0B` | `PRESSURE_CLICK_L` / `_R` | none |
| `0x04` / `0x05` | move finished (response to `MOVE_*`) | 2 bytes: position reached (mm × 10 / deg × 10) |
| `0x0F` / `0x0D` / `0x0E` | `BREATHING_DETECTION_RELAXED` / `_NORMAL` / `_STRESSED` | 1 byte `00`; nothing is sent if there were fewer than two inhales |

`DISABLE_MOTORS` (`0x07`) and `BREATHING_RATE` (`0x09`) are in the protocol but
not handled by this firmware.

## Building and flashing

1. Install [PlatformIO](https://platformio.org/) (e.g. the VS Code extension).
2. Open this folder as a PlatformIO project.
3. Build and upload the environment `neffy_esp32` (the default):
   ```
   pio run -e neffy_esp32 -t upload
   ```

Everything board-specific is in the repository: the board definition
(`boards/neffy_mainboard.json`) and the pin variant (`variants/`). The libraries
(`Neffy_UART_Interface`, `TMX222x_StepperDriver`, FastLED) are fetched by
PlatformIO at fixed versions on the first build.

Upload works over the board's own USB port without pressing any button: the
upload reboots the chip into its bootloader, and `tools/reset_after_upload.py`
starts the firmware afterwards. A red "serial exception" line at the end of an
upload is harmless as long as it ends with `SUCCESS`.

The other `neffy_*_test` environments in `platformio.ini` belong to hardware
test sketches that are kept locally and are not part of this repository.

## Configuration

All pins, hardware constants, timings and LED behaviour are in
[`src/neffy_esp32/config.h`](src/neffy_esp32/config.h). The tuning constants of
the detectors stay next to their code, with comments on how they were found
(`PressureDetector.cpp`, `PersonSensor.cpp`, `BreathingDetection.cpp`).

## Debugging

Over the USB port (silent unless switched on, no reflash needed):

| Key | Effect | Read by |
|---|---|---|
| `U` / `u` | UART monitor on / off: every frame on the TX and RX pin, read back from the pins themselves | `tools/uart_monitor.py` (live view with command names and payload meaning) |
| `D` / `d` | pocket voltages every loop | `tools/pocket_log.py` (recording with markers) |
| `T` / `t` | thermal sensor readings | `tools/thermal_log.py` (guided recording) |
| `M` / `m` | speaker muted / sound on | |

Run the tools from the project folder with the Python that comes with PlatformIO:

```
"%USERPROFILE%\.platformio\penv\Scripts\python.exe" tools\uart_monitor.py
```

Only one program can use the USB port at a time: stop a running tool
(`Ctrl+C`) before uploading.

## Layout

```
src/neffy_esp32/      firmware
  main.cpp              setup, main loop, LEDs, debug commands
  config.h              pins, timings, LED settings
  MotorControlLoop.*    motors on core 0: homing, MOVE_* commands, responses
  PersonDetector.*      sends the person state over UART
  PersonSensor.*        D6T thermal sensor and person detection
  PressureDetector.*    click detection on one pocket
  BreathingDetection.*  30 s breathing exercise
  VoltageReader.*       calibrated ADC readings
boards/, variants/    board definition and pin variant for PlatformIO
tools/                upload helper and debug tools (Python)
```

The motors run in their own task on core 0; UART, sensors and LEDs on core 1,
because the stepper interrupt and the LED output must not share a core.
