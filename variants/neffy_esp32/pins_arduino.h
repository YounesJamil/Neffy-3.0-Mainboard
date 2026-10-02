#ifndef Pins_Arduino_h
#define Pins_Arduino_h

#include <stdint.h>

#define USB_VID            0x303A
#define USB_PID            0x81BB
#define USB_MANUFACTURER   "Charly"
#define USB_PRODUCT        "Neffy-Mainboard"
#define USB_SERIAL         ""

static const uint8_t ALARM = -1;
static const uint8_t INT =    5;

static const uint8_t LED =  34;
static const uint8_t BTN =   0;

static const uint8_t TX =    43;
static const uint8_t RX =    44;
#define TX1 TX
#define RX1 RX

static const uint8_t SS =    -1;
static const uint8_t MISO =  -1;
static const uint8_t MOSI =  -1;
static const uint8_t SCK =   -1;

static const uint8_t SCL =   38;
static const uint8_t SDA =   37;

static const uint8_t SCL1 =  -1;
static const uint8_t SDA1 =  -1;

#define WIRE1_PIN_DEFINED 0
//static const uint8_t SCL1 =  48;
//static const uint8_t SDA1 =  47;

static const uint8_t A13 = 13;
static const uint8_t A14 = 14;

//Stepper Motor Pin Config
static const uint8_t STEP_HEAD =  4;
static const uint8_t DIR_HEAD  =  6;
static const uint8_t BTN_HEAD  = 11;
static const uint8_t EN_HEAD   =  5;

static const uint8_t STEP_BODY =  7;
static const uint8_t DIR_BODY  =  9;
static const uint8_t BTN_BODY  = 12;
static const uint8_t EN_BODY   =  8;

//Pin for muting the Speaker
static const uint8_t MUTE = 10;

#define LED_BUILTIN 34
#define BUILTIN_LED  LED_BUILTIN 

#endif /* Pins_Arduino_h */
