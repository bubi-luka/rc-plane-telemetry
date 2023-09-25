/* *************************************************************************** *
 * Project Title: RC plane telemetry
 * 
 * Description: A small Arduino project that uses various sensors to gather
 *              data and sends it back to the user radio control unit.
 *
 * Licence: GPL 3.0       (details in project file "LICENCE")
 * Version: 0.1.0         (details in project file "README.md")
 * created: 25. 09. 2023
 * by:      Luka Oman
 * *************************************************************************** */

// Declare additional libraries
#include <IBusBM.h>  // Library for iBus telemetry

// Define Arduino pins

// Define new objects
IBusBM iBusSensor;  // iBus telecomunication for sensors

// Define variables

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
}

// Main function, run in a loop
void loop() {

  // Send gathered data to the remote control unit on the ground
  //*****************************************************************************
  iBusSensor.setSensorMeasurement(1, 655);     // Temperature				[°C]  (400 + temp * 10)
  iBusSensor.setSensorMeasurement(2, 753);     // External Voltage	[V]   ()
  iBusSensor.setSensorMeasurement(3, 951);  // Pressure					[hPa] ()
  iBusSensor.setSensorMeasurement(4, 47);   // Relative altitude	[m]   ()
  iBusSensor.setSensorMeasurement(5, 0);       // Azimuth						[°]   ()
  iBusSensor.setSensorMeasurement(6, 17);    // Speed							[m/s] ()

  // send data to iBus
  iBusSensor.loop();
  //*****************************************************************************
}
