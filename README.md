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
| **Motors** (body = linear axis, head = rotation; TMC2225 stepper drivers) | Home against their endstops at every start (an axis already sitting on its endstop first moves off it), then only move on `MOVE_*` commands. Each finished move is answered with the position reached. |
| **LEDs** (WS2812B: a strip under each pocket, a ring of 12) | Follow the head: dim when it is down, full when it reaches its wake-up position, over the whole move. During breathing detection all LEDs grow brighter while inhaling. |
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
| `0x07` | `DISABLE_MOTORS` | 1 byte (not used). Sent once when a guided breathing exercise starts: the next `MOVE_*` starts 0.5 s later (and runs 0.5 s shorter, so it ends on time), to match the screen animation. Sent twice in a row when the exercise ends: cancels the delay. The motors stay on. |
| `0x10` | `BREATHING_DETECTION_START` | 1 byte `00`: start the 30 s breathing detection |

**Sent by the mainboard**

| ID | Message | Payload |
|---|---|---|
| `0x03` | `GET_PERSON_READING` | 1 byte: `01` = someone there, `00` = nobody |
| `0x08` / `0x0B` | `PRESSURE_CLICK_L` / `_R` | none |
| `0x04` / `0x05` | move finished (response to `MOVE_*`) | 2 bytes: position reached (mm × 10 / deg × 10) |
| `0x0F` / `0x0D` / `0x0E` | `BREATHING_DETECTION_RELAXED` / `_NORMAL` / `_STRESSED` | 1 byte `00`; nothing is sent if there were fewer than two inhales |

`BREATHING_RATE` (`0x09`) is in the protocol but not handled by this firmware.

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

### Watching the UART traffic (Windows cmd)

1. Connect the mainboard over USB with the firmware running (it shows up as a
   COM port; the tool finds it by itself).
2. Open `cmd` and change into the project folder:
   ```
   cd "C:\path\to\Neffy-3.0-Mainboard"
   ```
3. Start the monitor with the Python that comes with PlatformIO:
   ```
   "%USERPROFILE%\.platformio\penv\Scripts\python.exe" tools\uart_monitor.py
   ```
4. Every message between the two boards appears live, with time, direction,
   raw bytes, command name and what the payload means:
   ```
   18:31:12  tx  FA 03 01 01                       GET_PERSON_READING      someone there
   18:31:12  rx  FA 06 08 00 00 00 28 00 C8 00 28  MOVE_BODY_HEAD_IN_TIME  body to 0.0 mm in 4.0 s, head to 20.0 deg in 4.0 s
   18:31:16  tx  FA 05 02 00 C8                    MOVE_HEAD_IN_TIME       reached 20.0 deg
   18:31:20  tx  FA 0B 00                          PRESSURE_CLICK_R
   ```
   `tx` = sent by the mainboard, `rx` = received from the other board. The
   person reading repeats every 500 ms, so only its changes are shown; the full
   log, repeats included, is saved to `logs/uart-<date>.txt`.
5. Stop with `Ctrl+C`.

The other tools are started the same way (`tools\pocket_log.py`,
`tools\thermal_log.py`). Only one program can use the USB port at a time:
stop a running tool before uploading.

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
src/tests/            hardware test sketches (see Tests)
boards/, variants/    board definition and pin variant for PlatformIO
tools/                upload helper and debug tools (Python)
```

The motors run in their own task on core 0; UART, sensors and LEDs on core 1,
because the stepper interrupt and the LED output must not share a core.

## Tests

Small standalone sketches in `src/tests/` for checking single parts of the
hardware without the full firmware. Each has its own PlatformIO environment;
flash one, check the part, then flash `neffy_esp32` again.

| Environment | Sketch | Checks |
|---|---|---|
| `neffy_blink_test` | `blink` | The board is alive: the onboard LED blinks, "PIN HIGH / LOW" over USB |
| `neffy_led_test` | `led` | LED wiring: the ring lights up in full yellow |
| `neffy_led_fade_test` | `led_fade` | LED brightness control: the ring fades in and out |
| `neffy_stepper_test` | `stepper` | Body axis: moves 20 mm back and forth, LEDs follow the position |
| `neffy_stepper_head_test` | `stepper_head` | Head axis: homes, then moves up and down, LEDs follow the position |
| `neffy_endstop_test` | `endstops` | Endstop switches: shows which switch (GPIO 11 / 12) is pressed when pressed by hand; nothing moves |
| `neffy_pressure_click_test` | `pressure_click` | Pockets: the firmware's click detection with light response (a click flashes its pocket's LEDs), plus both pocket voltages every 20 ms (record with `tools/pocket_log.py`) |
| `neffy_person_sensor_test` | `person_sensor` | Thermal sensor: raw readings; read interval and I2C clock switchable over USB |
| `neffy_person_command_test` | `person_command` | The other board's reaction: sends `GET_PERSON_READING` 0 / 1 alternately every 5 s, without the sensor |

```
pio run -e neffy_stepper_head_test -t upload
```
