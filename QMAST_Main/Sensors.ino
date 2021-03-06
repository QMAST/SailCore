/*
  Sensors
  Handles updating and parsing of sensor inputs

  Created in January 2018, QMAST
*/
// TODO: Test GPS output format
// TODO: Implement Pixy and LIDAR

#include "pins.h"
#define DEFAULT_SENSOR_TRANSMILLIS 3000 // Default interval the Mega transmits sensor data
#define WIND_SPEED_VOLTAGE_CONVERSION_CONSTANT .004882814

// GPS definitions
#include <Adafruit_GPS.h>
Adafruit_GPS GPS(&SERIAL_PORT_GPS);
// Turns off echoing of GPS to Serial1
#define GPSECHO false

// Compass definitions
#include <Wire.h>
#define CMPS11_ADDRESS 0x60  // Address of CMPS11 shifted right one bit for arduino wire library
#define ANGLE_8  1           // Register to read 8bit angle from
unsigned char high_byte, low_byte, angle8;
char pitch, roll;
unsigned int angle16, temp16;

// Wind vane calibration variables
#define WINDVANE_LOW 0
#define WINDVANE_HIGH 1023

unsigned long lastSensorMillis; // Last time sensor data was parsed
unsigned long waitTimerMillis; // use this to break out of blocking tasks

// Storage for sensor data and interval of data transmission
String sensorCodes[] = {"GP", "CP", "TM", "WV", "PX", "LD", "SP", "WS"};
#define NUMBER_OF_CODES 8
String sensorStates[NUMBER_OF_CODES];
int sensorTransIntervalXBee[NUMBER_OF_CODES];
int sensorTransIntervalRPi[NUMBER_OF_CODES];
unsigned long sensorLastTransXBee[NUMBER_OF_CODES];
unsigned long sensorLastTransRPi[NUMBER_OF_CODES];

void initSensors() {
  // Set default sensor transmission intervals and sensor states
  for (int i = 0; i < NUMBER_OF_CODES; i++) {
    sensorTransIntervalXBee[i] = DEFAULT_SENSOR_TRANSMILLIS;
    sensorTransIntervalRPi[i] = DEFAULT_SENSOR_TRANSMILLIS;
    sensorStates[i] = "";
  }

  //Initialize Compass
  Wire.begin(); // Initiate the Wire library

  // Initialize GPS
  GPS.begin(9600);
  // Turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate, don't go above 1Hz to prevent overload
}

void checkSensors() {
  // Update sensor states if new information has arrived
  // Only runs once every 40 milliseconds, or about 25 times a second
  if (millis() - lastSensorMillis >= 50 && rcEnabled) {
    // Update compass
    Wire.beginTransmission(CMPS11_ADDRESS);  //starts communication with CMPS11
    Wire.write(ANGLE_8);                     //Sends the register we wish to start reading from
    Wire.endTransmission();
    // Request 5 bytes from the CMPS11
    // this will give us the 8 bit bearing,
    // both bytes of the 16 bit bearing, pitch and roll
    Wire.requestFrom(CMPS11_ADDRESS, 5);
    waitTimerMillis = millis();
    bool dataAvailable = true;
    while (Wire.available() < 5) {
      // Wait for all bytes to come back but if it takes longer than 500ms ignore sensor
      if (millis() - waitTimerMillis >= 30) {
        dataAvailable = false;
        break;
      }
    }
    if (dataAvailable) {
      angle8 = Wire.read();               // Read back the 5 bytes
      high_byte = Wire.read();
      low_byte = Wire.read();
      pitch = Wire.read();
      roll = Wire.read();
      angle16 = high_byte;                 // Calculate 16 bit angle
      angle16 <<= 8;
      angle16 += low_byte;
      setSensor("CP", String(angle16 / 10));

      // If the previous wire transmission didn't work, don't try again
      // Update Temperature
      Wire.beginTransmission(CMPS11_ADDRESS);  //starts communication with CMPS11
      Wire.write(24);                     //Sends the register we wish to start reading from
      Wire.endTransmission();
      Wire.requestFrom(CMPS11_ADDRESS, 2);
      waitTimerMillis = millis();
      dataAvailable = true;
      while (Wire.available() < 2) {
        // Wait for all bytes to come back but if it takes longer than 500ms ignore sensor
        if (millis() - waitTimerMillis >= 100) {
          dataAvailable = false;
          break;
        }
      }
      if (dataAvailable) {
        high_byte = Wire.read();
        low_byte = Wire.read();
        temp16 = high_byte;                 // Calculate 16 bit temperature
        temp16 <<= 8;
        temp16 += low_byte;

        setSensor("TM", String(21.0 + (float) temp16 / 8.)); // Calculate the temperature (8LSB/C) + guess intercept
      }
    }

    // Update GPS
    if (GPS.fix) {
      String location = "";
      if (GPS.lat == 'S') location = "-";
      location = location + String(GPS.lat) + ",";
      if (GPS.lon == 'E') location = location + "-";
      location = location + String(GPS.lon);
      setSensor("GP", location);
      String gpsSpeed = "" + String(GPS.speed);
      setSensor("SP", gpsSpeed); // Speed in knots
    } else {
      setSensor("SP", "0");
      setSensor("GP", "0");
    }

    // Update wind vane
    int angle = map(analogRead(APIN_WINDVANE), WINDVANE_LOW, WINDVANE_HIGH, 0, 360);
    setSensor("WV", String(angle));

    // Update wind speed
    float voltage = analogRead(APIN_WINDSPEED) * WIND_SPEED_VOLTAGE_CONVERSION_CONSTANT;
    float windSpeed = 32.4 * (voltage - 0.4) / 1.6;

    //DEBUG_PRINTLN(voltage);

    if (voltage >= 0.4) {
      setSensor("WS", String(windSpeed));
    } else {
      setSensor("WS", "0");
    }

    lastSensorMillis = millis();
  }

  // GPS data is can come in at any time (~1Hz) so keep looking for it to prevent dropping a message
  GPS.read();
  if (GPS.newNMEAreceived()) GPS.parse(GPS.lastNMEA());
}

void setSensor(String code, String data) {
  // Used to update data stored for a sensor
  for (int i = 0; i < NUMBER_OF_CODES; i++) {
    // Cycle through all of the sensor codes to find the one needing updating
    if (code.equals(sensorCodes[i])) {
      // Update the sensor storage
      sensorStates[i] = data;
      break;
    }
  }
}

String getSensor(String code, String data) {
  // Takes the sensor code and matches it with the appropriate data store
  // Used to update data stored for a sensor
  for (int i = 0; i < NUMBER_OF_CODES; i++) {
    // Cycle through all of the sensor codes to find the one requested
    if (code.equals(sensorCodes[i])) {
      // Update the sensor storage
      return sensorStates[i];
    }
  }
  return "";
}

void sendSensors() {
  // Check if the transmission interval/delay has passed for each sensor for the XBee + RPi
  // If the interval has passed, send the updated data
  unsigned long currentMillis = millis();
  for (int i = 0; i < NUMBER_OF_CODES; i++) {
    if (abs(currentMillis - sensorLastTransXBee[i]) >= sensorTransIntervalXBee[i] && sensorTransIntervalXBee[i] != 0 && !sensorStates[i].equals("")) {
      sendTransmission(PORT_XBEE, sensorCodes[i], sensorStates[i]);
      sensorLastTransXBee[i] = currentMillis;
    }
    if (abs(currentMillis - sensorLastTransRPi[i]) >= sensorTransIntervalRPi[i] && sensorTransIntervalRPi[i] != 0 && !sensorStates[i].equals("")) {
      sendTransmission(PORT_RPI, sensorCodes[i], sensorStates[i]);
      sensorLastTransRPi[i] = currentMillis;
      DEBUG_PRINT(sensorCodes[i]);
      DEBUG_PRINT(F(": "));
      DEBUG_PRINTLN(sensorStates[i]);
    }
  }
}

void setSensorTransInterval(int port, String code, int interval) {
  // Called when the XBee/RPi requests to be updated with sensor data at a different frequency
  for (int i = 0; i < NUMBER_OF_CODES; i++) {
    // Cycle through all of the sensor codes to find the one needing updating
    if (code.equals(sensorCodes[i])) {
      // Update the sensor transmission interval for the device sending the request
      if (port == PORT_XBEE) {
        if (interval != 0 && interval < 250) interval = 250; // After testing, the GUI does not respond to absurdly fast updates
        sensorTransIntervalXBee[i] = interval;
      }
      if (port == PORT_RPI) sensorTransIntervalRPi[i] = interval;
      break;
    }
  }
}
