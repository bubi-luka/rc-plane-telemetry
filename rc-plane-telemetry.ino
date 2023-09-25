/* *************************************************************************** *
 * Project Title: RC plane telemetry
 * 
 * Description: A small Arduino project that uses various sensors to gather
 *              data and sends it back to the user radio control unit.
 *
 * Licence: GPL 3.0       (details in project file "LICENCE")
 * Version: 0.3.0         (details in project file "README.md")
 * created: 25. 09. 2023
 * by:      Luka Oman
 * *************************************************************************** */

// Declare additional libraries
#include <IBusBM.h>           // Library for iBus telemetry
#include <Adafruit_BMP280.h>  // Library for BMP-280 senson

// Define Arduino pins
static const int voltagePin = A3;  // Analog 3 => for voltage data

// Define new objects
IBusBM iBusSensor;             // iBus telecomunication for sensors
Adafruit_BMP280 sensorBMP280;  // pressure, temperature, altitude

// Define variables
int inputVoltage = 0;  // battery voltage
int baseAltitude = 0;  // altitude at the start
int altitude = 0;      // altitude - from barometer

// Define custom functions

// Initialization function, run only once
void setup() {

  // start iBus comunication
  //*****************************************************************************
  iBusSensor.begin(Serial);  // iBusSensor.begin(Serial, IBUSBM_NOTIMER);

  iBusSensor.addSensor(0x01);  // Temperature				[°C]  (400 + temp * 10)
  iBusSensor.addSensor(0x03);  // External Voltage	[V]   ()
  iBusSensor.addSensor(0x41);  // Pressure					[hPa] ()
  iBusSensor.addSensor(0xfd);  // Relative altitude	[m]   ()
  iBusSensor.addSensor(0xfd);  // Azimuth						[°]   ()
  iBusSensor.addSensor(0xfd);  // Speed							[m/s] ()
  //*****************************************************************************

  // start BMP-280 sensor - altimeter
  //*****************************************************************************
  sensorBMP280.begin(0x76);

  // get the altitude at the starting point
  for (int i = 0; i < 15; i++) {
    baseAltitude += int(sensorBMP280.readAltitude());
    delay(250);
  }
  baseAltitude = baseAltitude / 15;
  //*****************************************************************************
}

// Main function, run in a loop
void loop() {

  // Get external battery voltage
  //*****************************************************************************
  float inVol = 0.0;
  inVol = analogRead(voltagePin);           // value we measured
  inVol = inVol * 4.9;                      // value we got from calibration
  inVol = inVol / 1024;                     // the set value
  inVol = inVol / (10.0 / (100.0 + 10.0));  // values from resistors in kOhm
  inputVoltage = inVol * 100;               // convert from float to int
  //*****************************************************************************

  // Get data from barometer
  //*****************************************************************************
  altitude = int(sensorBMP280.readAltitude()) - baseAltitude;
  //*****************************************************************************

  // Send gathered data to the remote control unit on the ground
  //*****************************************************************************
  iBusSensor.setSensorMeasurement(1, 655);                                // Temperature				[°C]  (400 + temp * 10)
  iBusSensor.setSensorMeasurement(2, inputVoltage);                       // External Voltage	[V]   ()
  iBusSensor.setSensorMeasurement(3, sensorBMP280.readPressure() / 100);  // Pressure					[hPa] ()
  iBusSensor.setSensorMeasurement(4, altitude);                           // Relative altitude	[m]   ()
  iBusSensor.setSensorMeasurement(5, 0);                                  // Azimuth						[°]   ()
  iBusSensor.setSensorMeasurement(6, 17);                                 // Speed							[m/s] ()

  // send data to iBus
  iBusSensor.loop();
  //*****************************************************************************
}
