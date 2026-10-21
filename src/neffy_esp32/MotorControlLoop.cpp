#include "MotorControlLoop.h"
#include "config.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"

QueueHandle_t motorQueueBody = NULL;
QueueHandle_t motorQueueHead = NULL;

volatile bool  motorsHomed     = false;
volatile float headPositionDeg = 0.0f;
volatile float headTopDeg      = BoardConfig::MAX_POSITION_HEAD_DEG / 2.0f; // until the first upward move

// Remembers the target of an upward head move (see headTopDeg).
static void noteHeadTarget(float targetDeg) {
    if (targetDeg > headPositionDeg + 0.5f) headTopDeg = fminf(targetDeg, BoardConfig::MAX_POSITION_HEAD_DEG);
}

// Position for a MOVE_* response: rounded (19.99 deg must report 20.0, not 19.9)
// and never below 0, which would wrap around as an unsigned number.
static inline uint16_t responseValue(float position, float scaleFactor) {
    return static_cast<uint16_t>(lroundf(fmaxf(position, 0.0f) * scaleFactor));
}

// Delay of the first move after DISABLE_MOTORS (see MOVE_DELAY_AFTER_DISABLE_MS).
// Moves arriving within that delay wait for the same moment, so a move split into
// a body and a head command stays in step. Set and read on core 1 (UART callbacks).
static volatile bool     moveDelayArmed = false;
static volatile uint32_t moveDelayUntil = 0;

void disableMotors(Message&) { moveDelayArmed = true; }

// Start time for a move received now: 0 = at once.
static uint32_t moveStartMs() {
    uint32_t now = millis();
    if(moveDelayArmed) {
        moveDelayArmed = false;
        moveDelayUntil = now + BoardConfig::MOVE_DELAY_AFTER_DISABLE_MS;
    }
    if((int32_t)(moveDelayUntil - now) > 0) return moveDelayUntil;
    return 0;
}

// Two payload bytes as one number, high byte first (e.g. 00 C8 = 200).
static inline uint16_t readU16(const Message& msg, uint8_t i) {
    return ((uint16_t)msg.buffer[i] << 8) | msg.buffer[i + 1];
}

void moveBody(Message& msg) {

    if(msg.bytes_in_buffer < NeffyCommands::MOVE_BODY_IN_TIME.payloadLength) return;

    ExchangeMotorData cmd;
    cmd.startAtMs = moveStartMs();

    cmd.position = readU16(msg, 0) * NeffyCommands::MOVE_BODY_IN_TIME.scaleFactor;
    cmd.duration = readU16(msg, 2) * NeffyCommands::MOVE_BODY_IN_TIME.scaleFactor;

    if(motorQueueBody != NULL) xQueueOverwrite(motorQueueBody, &cmd);

    return;
}

void moveHead(Message& msg) {

    if(msg.bytes_in_buffer < NeffyCommands::MOVE_HEAD_IN_TIME.payloadLength) return;

    ExchangeMotorData cmd;
    cmd.startAtMs = moveStartMs();

    cmd.position = readU16(msg, 0) * NeffyCommands::MOVE_HEAD_IN_TIME.scaleFactor;
    cmd.duration = readU16(msg, 2) * NeffyCommands::MOVE_HEAD_IN_TIME.scaleFactor;

    noteHeadTarget(cmd.position);
    if(motorQueueHead != NULL) xQueueOverwrite(motorQueueHead, &cmd);

    return;
}

void moveBodyAndHead(Message& msg) {

    if(msg.bytes_in_buffer < NeffyCommands::MOVE_BODY_HEAD_IN_TIME.payloadLength) return;

    ExchangeMotorData cmd;
    cmd.startAtMs = moveStartMs();   // same moment for body and head

    cmd.position = readU16(msg, 0) * NeffyCommands::MOVE_BODY_IN_TIME.scaleFactor;
    cmd.duration = readU16(msg, 2) * NeffyCommands::MOVE_BODY_IN_TIME.scaleFactor;

    if(motorQueueBody != NULL) xQueueOverwrite(motorQueueBody, &cmd);

    cmd.position = readU16(msg, 4) * NeffyCommands::MOVE_HEAD_IN_TIME.scaleFactor;
    cmd.duration = readU16(msg, 6) * NeffyCommands::MOVE_HEAD_IN_TIME.scaleFactor;

    noteHeadTarget(cmd.position);
    if(motorQueueHead != NULL) xQueueOverwrite(motorQueueHead, &cmd);

    return;
}

// Moves an axis off its endstop if it is already pressed (switch active-low).
// The library only detects the switch being pressed (falling edge), so homing
// from a pressed switch never saw an edge, drove into the hard stop and lost
// steps. While moving away the endstop interrupt is masked, so a bounce on
// release cannot count as a hit; an edge latched meanwhile is dropped.
template <class Motor>
static void moveOffEndstop(Motor& motor, uint8_t btnPin, float away, float speed) {
    if(digitalRead(btnPin) != LOW) return;   // free: nothing to do

    gpio_intr_disable((gpio_num_t)btnPin);
    for(int tries = 0; tries < 3 && digitalRead(btnPin) == LOW; tries++) {

        motor.moveToPositionInTime(motor.getPosition() + away, fabsf(away) / speed);
        unsigned long startMs = millis();
        while(motor.update() != IDLE && millis() - startMs < 5000) {}
    }
    GPIO.status_w1tc = 1UL << btnPin;        // pins 11/12: status register for GPIO 0..31
    gpio_intr_enable((gpio_num_t)btnPin);
}

void motorCoreControlLoop(void* args) {

    //Task watchdog: 30 s timeout and no panic. Core 0's idle task is taken off the
    //watchdog in setup() (disableCore0WDT), since this loop never yields.
    esp_task_wdt_init(30, false);

    //Queues for Sharing Data with the Main Program
    motorQueueBody = xQueueCreate(1, sizeof(ExchangeMotorData));
    motorQueueHead = xQueueCreate(1, sizeof(ExchangeMotorData));

    //Motor Initialization
    ExchangeMotorData cmd;

    LinearActuator  motorBody(BoardConfig::STEPPER_MICROSTEPS,
                              BoardConfig::BODY_MM_PER_ROTATION,
                              BoardConfig::STEP_BODY_PIN,
                              BoardConfig::DIR_BODY_PIN,
                              BoardConfig::EN_BODY_PIN,
                              -1, -1,
                              BoardConfig::BTN_BODY_PIN);

    AngularActuator motorHead(BoardConfig::STEPPER_MICROSTEPS,
                              BoardConfig::HEAD_GEAR_RATIO,
                              BoardConfig::STEP_HEAD_PIN,
                              BoardConfig::DIR_HEAD_PIN,
                              BoardConfig::EN_HEAD_PIN,
                              -1, -1,
                              BoardConfig::BTN_HEAD_PIN);

    MotorController controller;
    MotorState      stateBody;
    MotorState      stateHead;

    Message responseBody;
            responseBody.command         = NeffyResponse::MOVE_BODY_IN_TIME.id;
            responseBody.bytes_in_buffer = NeffyResponse::MOVE_BODY_IN_TIME.payloadLength;

    Message responseHead;
            responseHead.command         = NeffyResponse::MOVE_HEAD_IN_TIME.id;
            responseHead.bytes_in_buffer = NeffyResponse::MOVE_HEAD_IN_TIME.payloadLength;

    bool body_motion_active = false;
    bool head_motion_active = false;

    ExchangeMotorData pendingBody, pendingHead;   //moves waiting for their start time
    bool bodyPending = false, headPending = false;

    //Add the receiving methods to the neffy interface
    interface.addMethod(NeffyCommands::MOVE_BODY_IN_TIME.id, &moveBody);
    interface.addMethod(NeffyCommands::MOVE_HEAD_IN_TIME.id, &moveHead);
    interface.addMethod(NeffyCommands::MOVE_BODY_HEAD_IN_TIME.id, &moveBodyAndHead);
    interface.addMethod(NeffyCommands::DISABLE_MOTORS.id, &disableMotors);

    //Add the motors to the controller so that they receive stepping pulses
    controller.addMotor(&motorBody);
    controller.addMotor(&motorHead);
    controller.begin();
    controller.enableStepping();

    //Away from the endstop = opposite of the homing direction
    motorHead.invertDirection(BoardConfig::HEAD_DIR_INVERTED);
    moveOffEndstop(motorBody, BoardConfig::BTN_BODY_PIN,
                   BoardConfig::HOME_DIR_BODY == 1 ? BoardConfig::HOME_CLEAR_BODY_MM : -BoardConfig::HOME_CLEAR_BODY_MM,
                   BoardConfig::HOMING_SPEED_BODY_MM_S);
    moveOffEndstop(motorHead, BoardConfig::BTN_HEAD_PIN,
                   BoardConfig::HOME_DIR_HEAD == 1 ? BoardConfig::HOME_CLEAR_HEAD_DEG : -BoardConfig::HOME_CLEAR_HEAD_DEG,
                   BoardConfig::HOMING_SPEED_HEAD_DEG_S);

    //Home both motors at the same time, then zero each one as soon as it is done.
    //home() must not be called again once it returned true: it would start a new
    //homing move.
    motorBody.configHoming(BoardConfig::HOME_DIR_BODY == 1 ? BACKWARD : FORWARD,
                           BoardConfig::HOMING_SPEED_BODY_MM_S,
                           BoardConfig::HOME_BACKOFF_BODY_MM,
                           BoardConfig::MAX_POSITION_BODY_MM);

    motorHead.configHoming(BoardConfig::HOME_DIR_HEAD == 1 ? BACKWARD : FORWARD,
                           BoardConfig::HOMING_SPEED_HEAD_DEG_S,
                           BoardConfig::HOME_BACKOFF_HEAD_DEG,
                           BoardConfig::MAX_POSITION_HEAD_DEG);

    bool bodyHomed = false;
    bool headHomed = false;
    while(!bodyHomed || !headHomed) {

        if(!bodyHomed && motorBody.home()) { motorBody.tare(); bodyHomed = true; }
        if(!headHomed && motorHead.home()) { motorHead.tare(); headHomed = true; }
    }

    motorsHomed = true;

    //Motor Control Loop
    while (true) {

        //A move waits until its start time (see moveStartMs); a newer one replaces it
        if(xQueueReceive(motorQueueBody, &cmd, 0) == pdTRUE) { pendingBody = cmd; bodyPending = true; }
        if(xQueueReceive(motorQueueHead, &cmd, 0) == pdTRUE) { pendingHead = cmd; headPending = true; }

        if(bodyPending && (pendingBody.startAtMs == 0 || (int32_t)(millis() - pendingBody.startAtMs) >= 0)) {

            motorBody.moveToPositionInTime(pendingBody.position, pendingBody.duration);
            body_motion_active = true;
            bodyPending        = false;
        }

        if(headPending && (pendingHead.startAtMs == 0 || (int32_t)(millis() - pendingHead.startAtMs) >= 0)) {

            motorHead.moveToPositionInTime(pendingHead.position, pendingHead.duration);
            head_motion_active = true;
            headPending        = false;
        }

        stateBody = motorBody.update();
        stateHead = motorHead.update();
        headPositionDeg = motorHead.getPosition();

        //Check if the motion was completed. If so, send the corresponding response
        if(body_motion_active && stateBody == IDLE) {

            uint16_t bodyPos = responseValue(motorBody.getPosition(), NeffyResponse::MOVE_BODY_IN_TIME.scaleFactor);
            responseBody.buffer[0] = bodyPos  >>  8;   // high byte first
            responseBody.buffer[1] = bodyPos & 0xFF;   // then low byte
            interface.writeMessage(responseBody);

            body_motion_active = false;
        }

        if(head_motion_active && stateHead == IDLE) {

            uint16_t headPos = responseValue(motorHead.getPosition(), NeffyResponse::MOVE_HEAD_IN_TIME.scaleFactor);
            responseHead.buffer[0] = headPos  >>  8;   // high byte first
            responseHead.buffer[1] = headPos & 0xFF;   // then low byte
            interface.writeMessage(responseHead);

            head_motion_active = false;
        }
    }
}
