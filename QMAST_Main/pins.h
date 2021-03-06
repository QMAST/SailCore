/*
  pins.h
  Header file mapping to the 2017-2018 boat

  Created in December 2017, QMAST
*/


#define DEBUG // Comment this line out to disable debug printing to USB serial

#ifdef DEBUG
#define DEBUG_PRINTLN(x)  SERIAL_PORT_CONSOLE.println (x)
#define DEBUG_PRINT(x)  SERIAL_PORT_CONSOLE.print (x)
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#endif

#ifndef _PINS_H
#define _PINS_H

// Console
#define SERIAL_PORT_CONSOLE         Serial2
#define SERIAL_BAUD_CONSOLE         57600

// XBee
#define SERIAL_PORT_XBEE            Serial1
#define SERIAL_BAUD_XBEE            57600

// Raspberry Pi
#define SERIAL_PORT_RPI             Serial
#define SERIAL_BAUD_RPI             9600

// GPS
#define SERIAL_PORT_GPS             Serial3
#define SERIAL_BAUD_GPS             9600

#define PORT_RPI                    1
#define PORT_XBEE                   2

// Remote control (Spektrum)
#define PIN_RC_CH1                  52
#define PIN_RC_CH2                  48
#define PIN_RC_CH3                  24
#define PIN_RC_CH4                  25
#define PIN_RC_CH5                  26
#define PIN_RC_CH6                  27
#define PIN_RC_CH7                  28
#define PIN_RC_CH8                  29

#define RC_STD_TIMEOUT 50000 // Time (micros) to wait for pulses to begin from the RC receiver, recommend at least 20000
#define RC_MIN_DELAY 30 // Minimum time (millis) between checking RC input
#define RC_SMOOTHING_CONS 0.2 //Time (millis) over which to perform exponential smoothing (tau value)

#define CHANNEL_RUDDERS PIN_RC_CH2 // Adjust this so the left/right motion of the right stick corresponds with rudder movement
#define RUDDER_PULSE_LOW 1300
#define RUDDER_PULSE_HIGH 1900
#define RUDDER_DEAD_WIDTH 20
#define RUDDER_CENTER (0.5 * RUDDER_PULSE_HIGH + 0.5 * RUDDER_PULSE_LOW)

#define CHANNEL_WINCH PIN_RC_CH1 // Adjust this so that the up/down motion of the left stick corresponds with the winch channel
#define WINCH_PULSE_LOW 1130
#define WINCH_PULSE_HIGH 1880
#define WINCH_UPPER_LIMIT 180 // Value sent to sailwinch corresponding to fully sheeted out (experimentally determined for 2018-2019 boat)
#define WINCH_LOWER_LIMIT 0 // Value sent to sailwinch corresponding to fully sheeted in (experimentally determined for 2018-2019 boat)

// Servo connections
#define PIN_SERVO_1                 8
#define PIN_SERVO_2                 9
#define PIN_SERVO_3                 10
#define PIN_SERVO_WINCH             7

// Wind Vane (Analog)
#define APIN_WINDVANE               1
#define APIN_WINDSPEED              2

// Compass (CMPS11)
#define COMPASS_ADDRESS             0xC0

#endif
