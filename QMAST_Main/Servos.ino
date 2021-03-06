/*
  Servos
  Code controlling the movement of the rudder, winch and any other servos

  Created in January 2018, QMAST
*/

#include "pins.h"
#include <Servo.h>

Servo servoRudder;
Servo servoWinch;

int curWinchPos; // Store the current winch position

void initServos() {
  servoRudder.attach(PIN_SERVO_1);
  servoWinch.attach(PIN_SERVO_WINCH);
}

void moveRudder(int pos) {
  // Move the rudder between 0-180
  pos = constrain(pos, 0, 180);
  //DEBUG_PRINT(F("Moving rudder to "));
  //DEBUG_PRINTLN(pos);
  servoRudder.write(pos);
}

void moveWinch(int pos) {
  // Move the winch between 0-180 (fully in vs fully out)
  pos = constrain(pos, 0, 180);
  // Experimentally limit the sail winch range of motion depending on sail winding
  pos = map(pos, 180, 0, WINCH_UPPER_LIMIT, WINCH_LOWER_LIMIT);
  if (abs(pos - curWinchPos) > 2) { // Don't allow winch to twitch with RC noise
    servoWinch.write(pos);
    curWinchPos = pos;
  }
}
