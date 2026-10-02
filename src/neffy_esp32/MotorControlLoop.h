#pragma once
#include "Arduino.h"
#include "MotorController.h"
#include "Neffy_Interface.h"
#include <esp_task_wdt.h>         
#include <freertos/FreeRTOS.h>    
#include <freertos/queue.h>

extern QueueHandle_t motorQueueBody;
extern QueueHandle_t motorQueueHead;
extern NeffyInterface interface;

// Published by the control loop so core 1 can mirror the motion (e.g. LEDs).
// headPositionDeg stays 0 until homing is done. headTopDeg is the target of the
// last upward head move: LEDs reach full brightness exactly when the head gets there.
extern volatile bool  motorsHomed;
extern volatile float headPositionDeg;
extern volatile float headTopDeg;

struct ExchangeMotorData{

    float position = 0;
    float duration = 0;
};

/**
 * @brief Transmits the body motor control instructions to the control loop on core 0
 * @param msg Message struct that was received over UART
 */
void moveBody(Message& msg);

/**
 * @brief Transmits the head motor control instructions to the control loop on core 0
 * @param msg Message struct that was received over UART
 */
void moveHead(Message& msg);

/**
 * @brief Transmits the body and head motor control instructions to the control loop on core 0
 * @param msg Message struct that was received over UART
 */
void moveBodyAndHead(Message& msg);

/**
 * Homes both motors, then runs MOVE_* commands from the queues. Runs as its own
 * task on core 0 and never yields; all data shared with core 1 must be protected
 * from race conditions (queues, volatile flags).
 *
 * Start the task in setup():
 *  - `xTaskCreatePinnedToCore(&motorCoreControlLoop, "MotorLoop", 4096, nullptr, 2, nullptr, 0)`
 */
void motorCoreControlLoop(void* args);