#include "MotorControlLoop.h"
#include "config.h"

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

// Two payload bytes as one number, high byte first (e.g. 00 C8 = 200).
static inline uint16_t readU16(const Message& msg, uint8_t i) {
    return ((uint16_t)msg.buffer[i] << 8) | msg.buffer[i + 1];
}

void moveBody(Message& msg) {

    if(msg.bytes_in_buffer < NeffyCommands::MOVE_BODY_IN_TIME.payloadLength) return;

    ExchangeMotorData cmd;

    cmd.position = readU16(msg, 0) * NeffyCommands::MOVE_BODY_IN_TIME.scaleFactor;
    cmd.duration = readU16(msg, 2) * NeffyCommands::MOVE_BODY_IN_TIME.scaleFactor;

    if(motorQueueBody != NULL) xQueueOverwrite(motorQueueBody, &cmd);

    return;
}

void moveHead(Message& msg) {

    if(msg.bytes_in_buffer < NeffyCommands::MOVE_HEAD_IN_TIME.payloadLength) return;

    ExchangeMotorData cmd;

    cmd.position = readU16(msg, 0) * NeffyCommands::MOVE_HEAD_IN_TIME.scaleFactor;
    cmd.duration = readU16(msg, 2) * NeffyCommands::MOVE_HEAD_IN_TIME.scaleFactor;

    noteHeadTarget(cmd.position);
    if(motorQueueHead != NULL) xQueueOverwrite(motorQueueHead, &cmd);

    return;
}

void moveBodyAndHead(Message& msg) {

    if(msg.bytes_in_buffer < NeffyCommands::MOVE_BODY_HEAD_IN_TIME.payloadLength) return;

    ExchangeMotorData cmd;

    cmd.position = readU16(msg, 0) * NeffyCommands::MOVE_BODY_IN_TIME.scaleFactor;
    cmd.duration = readU16(msg, 2) * NeffyCommands::MOVE_BODY_IN_TIME.scaleFactor;

    if(motorQueueBody != NULL) xQueueOverwrite(motorQueueBody, &cmd);

    cmd.position = readU16(msg, 4) * NeffyCommands::MOVE_HEAD_IN_TIME.scaleFactor;
    cmd.duration = readU16(msg, 6) * NeffyCommands::MOVE_HEAD_IN_TIME.scaleFactor;

    noteHeadTarget(cmd.position);
    if(motorQueueHead != NULL) xQueueOverwrite(motorQueueHead, &cmd);

    return;
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

    //Add the receiving methods to the neffy interface
    interface.addMethod(NeffyCommands::MOVE_BODY_IN_TIME.id, &moveBody);
    interface.addMethod(NeffyCommands::MOVE_HEAD_IN_TIME.id, &moveHead);
    interface.addMethod(NeffyCommands::MOVE_BODY_HEAD_IN_TIME.id, &moveBodyAndHead);

    //Add the motors to the controller so that they receive stepping pulses
    controller.addMotor(&motorBody);
    controller.addMotor(&motorHead);
    controller.begin();
    controller.enableStepping();

    //Home both motors at the same time, then zero each one as soon as it is done.
    //home() must not be called again once it returned true: it would start a new
    //homing move.
    motorBody.configHoming(BoardConfig::HOME_DIR_BODY == 1 ? BACKWARD : FORWARD,
                           BoardConfig::HOMING_SPEED_BODY_MM_S,
                           BoardConfig::HOME_BACKOFF_BODY_MM,
                           BoardConfig::MAX_POSITION_BODY_MM);

    motorHead.invertDirection(BoardConfig::HEAD_DIR_INVERTED);
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

        if(xQueueReceive(motorQueueBody, &cmd, 0) == pdTRUE) {

            motorBody.moveToPositionInTime(cmd.position,  cmd.duration);
            body_motion_active = true;
        }

        if(xQueueReceive(motorQueueHead, &cmd, 0) == pdTRUE) {

            motorHead.moveToPositionInTime(cmd.position,  cmd.duration);
            head_motion_active = true;
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
