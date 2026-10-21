#pragma once
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/uart.h"

/** Pins, hardware constants and timings of the Neffy ESP32 board (ESP32-S3). */
namespace BoardConfig {
    constexpr uint8_t    RX_PIN             = 44;
    constexpr uint8_t    TX_PIN             = 43;
    constexpr uint32_t   BAUDRATE           = 115200;
    constexpr gpio_num_t LED_PIN            = GPIO_NUM_34;
    constexpr uint8_t    PRESSURE_PIN_L     = 13;
    constexpr uint8_t    PRESSURE_PIN_R     = 14;
    constexpr uint8_t    PRESSURE_SAMPLES   = 8;     // ADC samples averaged per pocket reading

    // Speaker mute, set by the MUTE command (payload 1 = muted, 0 = sound on);
    // sound is on after start. LOW mutes the amplifier (checked on the board).
    constexpr uint8_t    MUTE_PIN           = 10;
    constexpr bool       MUTE_ACTIVE_HIGH   = false;

    // Pockets calibrate themselves (see PressureDetector). Set INVERT if a pocket
    // is assembled so that pressing lowers the voltage instead of raising it.
    constexpr bool       PRESSURE_INVERT_L  = false;
    constexpr bool       PRESSURE_INVERT_R  = false;
    // Fine-tuning on top of the self-calibration: 1.0 = as calibrated, below 1 a
    // click must be stronger (thresholds / sensitivity), above 1 weaker.
    constexpr float      PRESSURE_SENSITIVITY_L = 1.05f;
    constexpr float      PRESSURE_SENSITIVITY_R = 0.92f;

    // Stepper motors (TMC2225 via TMX222x_StepperDriver lib). STEP/DIR/EN, EN active-low;
    // BTN = homing endstop, active-low with pull-up. The MS pins are strapped on the
    // board, not routed to the ESP32, so this must match the board: 4 does (MOVE_*
    // commands reach the commanded positions). A wrong value scales every move.
    constexpr int     STEPPER_MICROSTEPS   = 4;      // 4 / 8 / 16 / 32
    constexpr uint8_t STEP_HEAD_PIN = 4;
    constexpr uint8_t DIR_HEAD_PIN  = 6;
    constexpr uint8_t EN_HEAD_PIN   = 5;
    constexpr uint8_t BTN_HEAD_PIN  = 11;

    constexpr uint8_t STEP_BODY_PIN = 7;
    constexpr uint8_t DIR_BODY_PIN  = 9;
    constexpr uint8_t EN_BODY_PIN   = 8;
    constexpr uint8_t BTN_BODY_PIN  = 12;

    // Body = linear axis: mm of travel per full motor revolution (leadscrew lead).
    constexpr float BODY_MM_PER_ROTATION   = 8.0f;   // 8 mm lead
    // Head = angular axis: motor revolutions per one output revolution.
    constexpr float HEAD_GEAR_RATIO        = 40.0f / 30.0f;  // 30T pinion -> 40T output gear

    // Homing per axis: travel range measured from the endstop, approach speed, and
    // the back-off kept as a safety margin after the switch is found.
    constexpr float  MAX_POSITION_BODY_MM    = 85.0f;
    // An axis that already sits on its endstop at power-up first moves this far
    // away (up to 3 times) before homing: the endstop is only detected when it
    // gets pressed, so homing from a pressed switch drove into the hard stop.
    constexpr float  HOME_CLEAR_BODY_MM      = 5.0f;
    constexpr float  HOMING_SPEED_BODY_MM_S  = 4.0f;
    constexpr float  HOME_BACKOFF_BODY_MM    = 2.0f;
    constexpr int8_t HOME_DIR_BODY           = 1;    // 1 = home BACKWARD, 0 = home FORWARD

    // Measured: ~34 deg of mechanical travel from the switch trigger point. The endstop
    // sits at the most forward position, so the back-off is dead zone stolen from the
    // forward lean. It must still be large enough to actually release the switch —
    // 0.25 deg rounded down to a single microstep (0.34 deg), the switch stayed pressed,
    // and homing hung in an endless hit/recover loop.
    constexpr float  HOME_BACKOFF_HEAD_DEG   = 2.0f;
    constexpr float  HOME_CLEAR_HEAD_DEG     = 5.0f;   // see HOME_CLEAR_BODY_MM
    constexpr float  MAX_POSITION_HEAD_DEG   = 32.0f; // back-off + range stays within the ~34 deg
    constexpr float  HOMING_SPEED_HEAD_DEG_S = 5.0f;
    constexpr int8_t HOME_DIR_HEAD           = 1;    // 1 = home BACKWARD, 0 = home FORWARD
    constexpr bool   HEAD_DIR_INVERTED       = false; // flips the head motor direction (depends on coil wiring)

    // The other board sends DISABLE_MOTORS once when a guided breathing exercise
    // starts and twice within ~0.3 s when it ends. The exercise's screen animation
    // appears about half a second after the first move command, so the first move
    // after a single DISABLE_MOTORS starts this much later (and runs this much
    // shorter, so it still ends on the other board's schedule). A second
    // DISABLE_MOTORS within DISABLE_PAIR_MS cancels it: the end of an exercise,
    // so the end pose and the next sleep / wake-up move are not delayed.
    // The motors are not switched off.
    constexpr uint32_t MOVE_DELAY_AFTER_DISABLE_MS = 500;
    constexpr uint32_t DISABLE_PAIR_MS             = 500;

    // Main loop on core 1 (UART, pockets, thermal sensor, LEDs).
    constexpr uint32_t LOOP_DELAY_MS         = 20;
    // The D6T delivers a new value about every 100 ms; polled faster (20 ms) it
    // stops measuring and returns the same value forever.
    constexpr uint32_t PERSON_READ_MS        = 100;
    // The person state is also repeated this often, so a lost message does not
    // go unnoticed until the next change.
    constexpr uint32_t PERSON_RESEND_MS      = 500;

    // Debug UART monitor ('U' over USB): two spare UARTs listen on the TX and RX
    // pin of the protocol UART.
    constexpr uart_port_t MONITOR_TX_UART    = UART_NUM_1;
    constexpr uart_port_t MONITOR_RX_UART    = UART_NUM_2;
}

/** WS2812B LED strips: one under each pocket, a ring of 12. */
namespace LedConfig {
    constexpr uint8_t POCKET_L_PIN     = 17;
    constexpr uint8_t POCKET_R_PIN     = 18;
    constexpr uint8_t RING_PIN         = 21;
    constexpr uint8_t POCKET_LEDS      = 8;      // LEDs per pocket strip
    constexpr uint8_t RING_LEDS        = 12;

    constexpr uint8_t COLOR_R = 255, COLOR_G = 160, COLOR_B = 0;  // dark, warm yellow
    constexpr uint8_t MIN_BRIGHT       = 50;     // idle glow when the head is down: 50 of 255 = ~20 %

    // Brightness follows the head over the whole move: down = MIN_BRIGHT, full
    // exactly when the head reaches the target of its last upward move (wake-up
    // and going to sleep take as long as the motor).
    // Breathing: while inhaling (pocket pressed) all strips rise to full within
    // BREATH_RISE_S and sink back within BREATH_FALL_S while exhaling.
    constexpr float   BREATH_RISE_S    = 6.0f;
    constexpr float   BREATH_FALL_S    = 2.0f;
}

/** D6T single-pixel thermal sensor I2C constants. */
namespace D6TConfig {
    constexpr uint8_t  I2C_ADDR  = 0x0A;
    constexpr uint8_t  I2C_CMD   = 0x4C;
    constexpr uint8_t  SDA_PIN   = 37;
    constexpr uint8_t  SCL_PIN   = 38;
    constexpr uint32_t I2C_CLOCK = 100000;
}
