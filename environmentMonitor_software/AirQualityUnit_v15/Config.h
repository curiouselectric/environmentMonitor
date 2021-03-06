#pragma once

// General Function Libraries - Already installed in Arduino IDE
#include <Arduino.h>
#include <stdint.h>
#include <Millis.h>
#include <EEPROM.h>        // For writing values to the EEPROM

//Adafruit IO info
//https://io.adafruit.com/api/docs/mqtt.html#mqtt-connection-details
#define AIO_SERVER                         "io.adafruit.com" // direct
#define AIO_SERVERPORT                     8883    // Use 8883 if at all possible! For secure conenction
#define AIO_USERNAME                      "*** YOUR ADAFRUIT IO USER NAME ***" // This is your Adafruit IO username
#define AIO_KEY                           "*** YOUR ADAFRUIT IO KEY ***" // This is your Adafruit IO Key



//// Thingspeak Info -Not yet implemented!!
//#define MQTT_SERVER                      "mqtt.thingspeak.com" // Via my thingspeak!
//#define MQTT_PORT                        1883
//#define MQTT_API                         "YOUR THINGSPEAK API KEY"
//#define MQTT_USER                        "Blergh" // Does not matter!


#define         DEBUG_GRAPH                0 // Show graph updates on serial
#define         DEBUG_MQTT                 1 // Show MQTT updates on serial
#define         DEBUG_TEMP                 0 // Show Temp updates on serial
#define         DEBUG_ENCODER              0 // Show Encoder updates on serial
#define         DEBUG_SDS                  0 // Show SDS dust sensor on serial
#define         DEBUG_BMP180               0 // Show BMP sensor on serial
#define         DEBUG_RAD                  0 // Show Radiation Data on serial

const uint8_t   SW_TX                      = D1;    // For software serial
const uint8_t   SW_RX                      = D2;    // For software serial

const uint8_t   SW_SDA                      = D7;    // Software I2C for sensors - also encoder!!
const uint8_t   SW_SCK                      = D6;    // Software I2C for sensors - also encoder!!

const uint8_t   SW_RX_RAD                  = D8;    // For reading the radiation sensor data
const uint8_t   SW_TX_RAD                  = D0;    // For reading the radiation sensor data (NOT USED?)

const uint8_t   ROT_A_PIN2                 = D7;
const uint8_t   ROT_B_PIN2                 = D6;
const uint8_t   ROT_PUSH_PIN2              = D8;

const uint8_t   CBOLED_SDA_PIN             = D5;
const uint8_t   CBOLED_SCK_PIN             = D4;
#define         CBOLED_CLASS               U8G2_SSD1306_128X64_NONAME_F_SW_I2C
#define         CBOLED_MESSAGE_FONT_8PT    u8g2_font_helvR08_tf
#define         CBOLED_MESSAGE_FONT_24PT   u8g2_font_luBS24_tn

const uint8_t   RGBLED_DATA_PIN            = D3;
const uint8_t   RGBLED_COUNT               = 22;
const uint8_t   RGBLED_BRIGHTNESS          = 50;
#define         RGBLED_TYPE                (NEO_GRB + NEO_KHZ800)

#define         DATA_AVERAGE               30  // The time between uploads in seconds
#define         GRAPH_AVERAGE              60 // The time between updates of the graph display in seconds
#define         DISPLAY_UPDATE             5000  // Time in mS between reading data and displaying new values

#define         SDS_WAKEUPTIME             30   // Number of seconds to run device before taking reading
#define         SDS_UPDATE                 180  // Number of seconds between reads of the SDS unit (wakes up SDS_WAKEUP before that)

#define         NUMBER_SENSORS             6
