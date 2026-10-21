// Neffy ESP32 firmware. The board does not decide anything itself:
//   - the thermal sensor state is sent out (GET_PERSON_READING: 0 = nobody,
//     1 = someone there) on every change and every 500 ms
//   - both pockets report clicks (PRESSURE_CLICK_L/R)
//   - BREATHING_DETECTION_START runs a 30 s breathing exercise on the pockets and answers
//     BREATHING_DETECTION_RELAXED / _NORMAL / _STRESSED; clicks pause meanwhile
//   - all motor moves arrive as MOVE_* commands and run in MotorControlLoop; the
//     first move after DISABLE_MOTORS (start of a guided breathing exercise)
//     starts 0.5 s later to match the screen animation
//   - MUTE switches the speaker (payload 1 = muted, 0 = sound on)
//   - the LEDs follow the real head position: down = faint glow, full when the
//     head reaches its wake-up target; during breathing detection all LEDs grow
//     brighter while a pocket is pressed
//
// Motors on core 0 (MotorControlLoop), UART + sensor + LEDs on core 1. The
// stepper's 50 kHz interrupt must not share a core with FastLED.
// Pins, timings and LED behaviour live in config.h.
//
// Debug over USB (silent unless asked):
//   'D' / 'd'  pocket stream on / off: "t_ms,L_V,R_V" per loop (read by
//              tools/pocket_log.py)
//   'T' / 't'  thermal stream on / off: "T,t_ms,temp_C,ptat_C,base_C,state" per
//              sensor read (read by tools/thermal_log.py)
//   'M' / 'm'  speaker muted / sound on (as the MUTE command)
//   'U' / 'u'  UART monitor on / off: every frame on the TX and RX pin, read
//              back from the pins themselves, as "tx|rx t_ms FA id len payload"
//              (read by tools/uart_monitor.py)

#include <Arduino.h>
#include <FastLED.h>
#include "driver/uart.h"
#include "esp_rom_gpio.h"
#include "soc/io_mux_reg.h"
#include "soc/uart_periph.h"
#include "Neffy_Interface.h"
#include "BreathingDetection.h"
#include "MotorControlLoop.h"
#include "PersonDetector.h"
#include "PressureDetector.h"
#include "VoltageReader.h"
#include "config.h"

NeffyInterface interface;                 // used by MotorControlLoop via extern
static PersonDetector   person(interface);
static VoltageReader    voltageL(BoardConfig::PRESSURE_PIN_L, BoardConfig::PRESSURE_SAMPLES);
static VoltageReader    voltageR(BoardConfig::PRESSURE_PIN_R, BoardConfig::PRESSURE_SAMPLES);
static PressureDetector pocketL(interface, voltageL, true,  BoardConfig::PRESSURE_INVERT_L, BoardConfig::PRESSURE_SENSITIVITY_L);
static PressureDetector pocketR(interface, voltageR, false, BoardConfig::PRESSURE_INVERT_R, BoardConfig::PRESSURE_SENSITIVITY_R);
static BreathingDetection breathing(interface);
static CRGB ledsPocketL[LedConfig::POCKET_LEDS];
static CRGB ledsPocketR[LedConfig::POCKET_LEDS];
static CRGB ledsRing[LedConfig::RING_LEDS];

static volatile bool breathingRequested = false;
static bool          streamPockets      = false;
static bool          monitorUart        = false;
static bool          streamThermal      = false;

// Runs inside interface.update(): only set a flag, the loop starts the session.
static void onBreathingStart(Message&) { breathingRequested = true; }

static void setSpeakerMuted(bool muted) {
    if (BoardConfig::MUTE_ACTIVE_HIGH) {
        digitalWrite(BoardConfig::MUTE_PIN, muted ? HIGH : LOW);
    } else {
        digitalWrite(BoardConfig::MUTE_PIN, muted ? LOW : HIGH);  // LOW mutes
    }
}

static void onMute(Message& msg) {
    if (msg.bytes_in_buffer >= 1) setSpeakerMuted(msg.buffer[0] == 0x01);  // 1 = mute, 0 = sound on
}

// level 0 = idle glow, 1 = full. Cubic because the eye perceives brightness
// logarithmically.
template <size_t N>
static void fillLevel(CRGB (&strip)[N], float level) {
    using namespace LedConfig;
    level = constrain(level, 0.0f, 1.0f);
    float   perceived  = level * level * level;
    uint8_t brightness = MIN_BRIGHT + (uint8_t)((255 - MIN_BRIGHT) * perceived);  // MIN_BRIGHT..255
    CRGB color(COLOR_R, COLOR_G, COLOR_B);
    color.nscale8(brightness);
    fill_solid(strip, N, color);
}

// Routes a pad into a spare UART's RX through the GPIO matrix. The pad keeps
// its job (TX output / RX input), so this shows what really is on the wire.
static void listenOnPad(uart_port_t uart, uint8_t pin) {
    uart_config_t cfg = {};
    cfg.baud_rate  = BoardConfig::BAUDRATE;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_APB;
    uart_driver_install(uart, 512, 0, 0, nullptr, 0);  // 512 B receive buffer, receive only
    uart_param_config(uart, &cfg);
    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[pin]);           // read the pad, its output stays as is
    esp_rom_gpio_connect_in_signal(pin, uart_periph_signal[uart].pins[SOC_UART_RX_PIN_IDX].signal, false);  // pad -> RX
}

static void startUartMonitor() {
    static bool installed = false;
    if (!installed) {
        listenOnPad(BoardConfig::MONITOR_TX_UART, BoardConfig::TX_PIN);
        listenOnPad(BoardConfig::MONITOR_RX_UART, BoardConfig::RX_PIN);
        installed = true;
    }
    uart_flush_input(BoardConfig::MONITOR_TX_UART);
    uart_flush_input(BoardConfig::MONITOR_RX_UART);
}

struct FrameReader {
    const char* dir;
    uint8_t     frame[3 + 16];  // header (FA, id, length) + up to 16 payload bytes
    uint8_t     have;
};

// Prints every complete frame a monitor UART has read.
static void printFrames(uart_port_t uart, FrameReader& r) {
    uint8_t in;
    while (uart_read_bytes(uart, &in, 1, 0) == 1) {
        if (r.have == 0 && in != START_OF_FRAME_IDENTIFIER) {
            Serial.printf("%s %lu stray %02X\n", r.dir, (unsigned long)millis(), in);
            continue;
        }
        r.frame[r.have++] = in;
        bool complete   = r.have >= 3 && r.have == 3 + r.frame[2];  // frame[2] = payload length
        bool bufferFull = r.have == sizeof(r.frame);
        if (complete || bufferFull) {
            Serial.printf("%s %lu", r.dir, (unsigned long)millis());
            for (uint8_t i = 0; i < r.have; i++) Serial.printf(" %02X", r.frame[i]);
            Serial.println();
            r.have = 0;
        }
    }
}

static void readDebugCommands() {
    while (Serial.available()) {
        switch (Serial.read()) {
            case 'D': streamPockets = true;  break;
            case 'd': streamPockets = false; break;
            case 'T': streamThermal = true;  break;
            case 't': streamThermal = false; break;
            case 'M': setSpeakerMuted(true);  break;
            case 'm': setSpeakerMuted(false); break;
            case 'U': startUartMonitor(); monitorUart = true; break;
            case 'u': monitorUart = false; break;
        }
    }
}

void setup() {
    // UART0 is the protocol line to the other board, and also where ESP-IDF
    // prints its log: keep it silent so only protocol frames go out.
    esp_log_level_set("*", ESP_LOG_NONE);
    Serial.begin(115200);                 // USB CDC, debug only: the baud rate is ignored
    FastLED.addLeds<WS2812B, LedConfig::POCKET_L_PIN, GRB>(ledsPocketL, LedConfig::POCKET_LEDS);
    FastLED.addLeds<WS2812B, LedConfig::POCKET_R_PIN, GRB>(ledsPocketR, LedConfig::POCKET_LEDS);
    FastLED.addLeds<WS2812B, LedConfig::RING_PIN,     GRB>(ledsRing,    LedConfig::RING_LEDS);
    fillLevel(ledsPocketL, 0.0f);
    fillLevel(ledsPocketR, 0.0f);
    fillLevel(ledsRing, 0.0f);
    FastLED.show();

    pinMode(BoardConfig::MUTE_PIN, OUTPUT);
    setSpeakerMuted(false);

    interface.initUART(BoardConfig::RX_PIN, BoardConfig::TX_PIN, BoardConfig::BAUDRATE, UART_NUM_0);
    person.init();
    voltageL.init();
    voltageR.init();
    pocketL.init();
    pocketR.init();
    interface.addMethod(NeffyCommands::BREATHING_DETECTION_START.id, &onBreathingStart);
    interface.addMethod(NeffyCommands::MUTE.id, &onMute);

    // MotorLoop owns core 0 and never yields, so the idle task there never runs.
    // Its watchdog then fired every 30 s and printed ~600 bytes onto UART0.
    disableCore0WDT();
    xTaskCreatePinnedToCore(&motorCoreControlLoop, "MotorLoop", 4096, nullptr, 2, nullptr, 0);
}

void loop() {
    static uint32_t lastSensorMs = 0;
    static uint32_t lastStatusMs = 0;

    interface.update();                   // MOVE_* commands -> MotorControlLoop queues
    pocketL.update(millis());             // sends clicks
    pocketR.update(millis());

    readDebugCommands();
    if (streamPockets) {
        Serial.printf("%lu,%.3f,%.3f\n", (unsigned long)millis(), pocketL.lastVoltage(), pocketR.lastVoltage());
    }

    if (breathingRequested) {
        breathingRequested = false;
        breathing.start(millis());
        pocketL.setMuted(true);
        pocketR.setMuted(true);
    }
    // update() returns true once, when the 30 s are over and the result is sent
    if (breathing.running() && breathing.update(millis(), pocketL.pressed() || pocketR.pressed())) {
        pocketL.setMuted(false);
        pocketR.setMuted(false);
    }

    if (millis() - lastSensorMs >= BoardConfig::PERSON_READ_MS) {
        lastSensorMs = millis();
        PersonState state = person.update();   // sends GET_PERSON_READING on change
        if (streamThermal) {
            Serial.printf("T,%lu,%.1f,%.1f,%.2f,%d\n", (unsigned long)millis(), person.temperature(),
                          person.ptat(), person.baseline(), (int)state);
        }
    }

    if (millis() - lastStatusMs >= BoardConfig::PERSON_RESEND_MS) {
        lastStatusMs = millis();
        person.sendState();
    }

    // LEDs: all strips follow the head position; breathing lifts them to full.
    using namespace LedConfig;
    static float    breath    = 0.0f;  // breathing boost, 0..1
    static uint32_t lastLedMs = 0;
    float dt     = (millis() - lastLedMs) / 1000.0f;  // seconds since the last loop
    lastLedMs    = millis();
    // 0 = head down .. 1 = head at its wake-up target; dark until homed
    float base = 0.0f;
    if (motorsHomed) base = constrain(headPositionDeg / fmaxf(headTopDeg, 1.0f), 0.0f, 1.0f);
    bool inhale  = breathing.running() && (pocketL.pressed() || pocketR.pressed());
    if (inhale && breath < base) breath = base;  // rise from what is lit now
    if (inhale) breath += dt / BREATH_RISE_S;  // inhaling: brighter
    else        breath -= dt / BREATH_FALL_S;  // exhaling: darker
    breath = constrain(breath, 0.0f, 1.0f);
    float level = fmaxf(base, breath);
    fillLevel(ledsPocketL, level);
    fillLevel(ledsPocketR, level);
    fillLevel(ledsRing,    level);
    FastLED.show();

    if (monitorUart) {
        static FrameReader tx = { "tx", {}, 0 }, rx = { "rx", {}, 0 };
        printFrames(BoardConfig::MONITOR_TX_UART, tx);
        printFrames(BoardConfig::MONITOR_RX_UART, rx);
    }
    delay(BoardConfig::LOOP_DELAY_MS);
}
