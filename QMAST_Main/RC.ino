/*
  RC
  Code managing remote control of rudder(s) and winch

  Created in January 2018, QMAST
*/
// TODO: Determine which RC channels to use
// TODO: Calibrate RC high/low pulses
#include "pins.h"

unsigned long lastRCMillis;

int oldWinchPos = 0; // variable to store previous winch position for exponential smoothing
int oldRudderPos = RUDDER_CENTER; // variable to store previous rudder position for exponential smoothing

bool rcEnabled = true;

void initRC() {
  pinMode(CHANNEL_RUDDERS, INPUT);
  pinMode(CHANNEL_WINCH, INPUT);
}

void setRCEnabled(bool state) {
  rcEnabled = state;
}

void updateRCWinch() {
  int winchPos = pulseIn(CHANNEL_WINCH, HIGH, RC_STD_TIMEOUT); // Get winch pulse value
  if (winchPos > 1000) {
    winchPos = smooth(winchPos, oldWinchPos); // Perform exponential smoothing to reduce twitching
    oldWinchPos = winchPos; // Save current winch position for future exponential smoothing
    winchPos = map(winchPos, WINCH_PULSE_HIGH, WINCH_PULSE_LOW, 180, 0); // Map the winch pulse value to a number between 0-180 for the servo library
    moveWinch(winchPos); // Move the winch
  } else {
    DEBUG_PRINTLN("RC Winch offline");
  }
}

void updateRCRudders() {
  int rudderPos = pulseIn(CHANNEL_RUDDERS, HIGH, RC_STD_TIMEOUT); // Get rudder pulse value
  if (rudderPos != 0) {
    rudderPos = smooth(rudderPos, oldRudderPos); // Perform exponential smoothing to reduce twitching
    oldRudderPos = rudderPos; // Save current rudder position for future exponential smoothing
    if (rudderPos <= (RUDDER_CENTER + RUDDER_DEAD_WIDTH / 2) && rudderPos >= (RUDDER_CENTER - RUDDER_DEAD_WIDTH / 2)) rudderPos = RUDDER_CENTER; // Catch dead-band signals
    rudderPos = map(rudderPos, WINCH_PULSE_HIGH, WINCH_PULSE_LOW, 180, 0); // Map the rudder pulse value to a number between 0-180 for the servo library
    moveRudder(rudderPos); // Move the rudder
  } else {
    DEBUG_PRINTLN(F("RC Rudder offline"));
  }
}

void checkRC() {
  // Wrapper function to improve code readability in the main loop and also rate limit RC updating
  if (millis() - lastRCMillis >= RC_MIN_DELAY && rcEnabled) {
    if (pulseIn(CHANNEL_RUDDERS, HIGH, RC_STD_TIMEOUT) > 2) {
      updateRCWinch();
      updateRCRudders();
    } else {
      DEBUG_PRINTLN(F("RC offline"));
    }
    lastRCMillis = millis();
  }
}

int smooth(int cur, int prev) {
  // Function that performs exponential smoothing given the current value, previous value, the time change and the time constant
  float smoothed = RC_SMOOTHING_CONS * cur + (1.0 - RC_SMOOTHING_CONS) * prev;
  return smoothed;
}
