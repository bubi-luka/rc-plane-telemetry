/* *************************************************************************** *
 * Project Title: RC plane telemetry
 * 
 * Description: A small Arduino project that uses various sensors to gather
 *              data and sends it back to the user radio control unit.
 *
 * Licence:   GPL 3.0       (details in project file "LICENCE")
 * Version:   0.9.9         (details in project file "README.md")
 * created:   25. 09. 2023
 * modified:  15. 01. 2024
 * by:        Luka Oman
 * *************************************************************************** */

// Declare additional libraries
#include <IBusBM.h>           // Library for iBus telemetry
#include <Adafruit_BMP280.h>  // Library for BMP-280  sensor
#include <QMC5883LCompass.h>  // Library for *MC-5883L sensor

// Define Arduino pins
static const int voltagePin = A3;   // Analog 3 => for voltage data
const uint8_t rpmInterruptPin = 0;  // on which pin the interrupt will be received
// static const int sdaPin = A4;    // Analog 4 => SDA pin for I2C - not used in the code
// static const int sclPin = A5;    // Analog 5 => SCL pin for I2C - not used in the code

// Define new objects
IBusBM iBusSensor;               // iBus telecomunication for sensors
Adafruit_BMP280 sensorBMP280;    // pressure, temperature, altitude
QMC5883LCompass sensorHMC5883L;  // compass

//Define constants
static const byte motorPoles = 14;    // number of magnets on motor
static const int rpmDelay = 250;      // delay timer for RPM reporting
static const int serialDelay = 1000;  // timer for serial console reporting

// Define variables
int inputVoltage = 0;           // battery voltage
int baseAltitude = 0;           // altitude at the start
int altitude = 0;               // altitude - from barometer
int azimuth = 0;                // azimuth - from compass
char direction[3];              // direction - from compass
unsigned long rpmTimer = 0;     // time of previously fired RPM interval
volatile int rpmCounter = 0;    // count the signals fired from RPM sensor
float rpmValue = 0;             // current rotations per minute
unsigned long serialTimer = 0;  // time of previous serial report

// Define custom functions

//  Function is called every time there is interrupt signal from RPM sensor
void rpmInterrupter() {
  rpmCounter++;
}


// Initialization function, run only once
void setup() {

  // Start communication protocols
  Serial.begin(9600);  // debugging
  Serial.println();    // start fresh on a new line

  // start iBus comunication
  //*****************************************************************************
  iBusSensor.begin(Serial);    // iBusSensor.begin(Serial, IBUSBM_NOTIMER);
  iBusSensor.addSensor(0x03);  // External Voltage	[V]   ()
  iBusSensor.addSensor(0x02);  // Speed							[m/s] ()
  iBusSensor.addSensor(0xfd);  // Relative altitude	[m]   ()
  iBusSensor.addSensor(0xfd);  // Azimuth						[°]   ()
  iBusSensor.addSensor(0x01);  // Temperature				[°C]  (400 + temp * 10)
  iBusSensor.addSensor(0x41);  // Pressure					[hPa] ()
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

  // start HMC-5883L - compass
  //*****************************************************************************
  sensorHMC5883L.init();
  //*****************************************************************************

  // start RPM sensor - interrupt signal code
  //*****************************************************************************
  pinMode(rpmInterruptPin, INPUT);
  attachInterrupt(rpmInterruptPin, rpmInterrupter, RISING);
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
  altitude = int(sensorBMP280.readAltitude()) - baseAltitude;  // value is the difference between base and current altitude
  //*****************************************************************************

  // Get data from compass
  //*****************************************************************************
  sensorHMC5883L.read();
  azimuth = sensorHMC5883L.getAzimuth();
  sensorHMC5883L.getDirection(direction, azimuth);
  //*****************************************************************************

  // Get data from RPM sensor
  //*****************************************************************************
  if (millis() - rpmTimer > rpmDelay) {
    rpmValue = ((float(rpmCounter * 2 / motorPoles)) / (millis() - rpmTimer)) * 1000 * 60;
    rpmCounter = 0;
    rpmTimer = millis();
  }
  //*****************************************************************************

  // Send gathered data to the remote control unit on the ground
  //*****************************************************************************
  iBusSensor.setSensorMeasurement(1, inputVoltage);                               // External Voltage	  [V]   ()
  iBusSensor.setSensorMeasurement(2, rpmValue);                                   // RPM							  []    ()
  iBusSensor.setSensorMeasurement(3, altitude);                                   // Relative altitude	[m]   ()
  iBusSensor.setSensorMeasurement(4, azimuth);                                    // Azimuth						[°]   ()
  iBusSensor.setSensorMeasurement(1, sensorBMP280.readTemperature() * 10 + 400);  // Temperature				[°C]  (400 + temp * 10)
  iBusSensor.setSensorMeasurement(3, sensorBMP280.readPressure() / 100);          // Pressure					  [hPa] ()
  iBusSensor.loop();                                                              // send data to iBus
  //*****************************************************************************

  // Save current data to the SD card every set interval
  //*****************************************************************************
  if (millis() - serialTimer >= serialDelay) {  // if the difference between previous time and current time is greater than set interval we write data to the card
    Serial.print(millis());
    Serial.print(F("\t"));
    Serial.print(float(inputVoltage) / 100, 2);
    Serial.print(F(" V\t"));
    Serial.print(rpmValue);
    Serial.print(F(" RPM\t"));
    Serial.print(sensorBMP280.readAltitude());
    Serial.print(F(" m\t"));
    Serial.print(altitude);
    Serial.print(F(" m\t"));
    Serial.print(sensorBMP280.readTemperature());
    Serial.print(F("° C\t"));
    Serial.print(sensorBMP280.readPressure() / 100);
    Serial.print(F(" hPa\t"));
    Serial.print(azimuth);
    Serial.print(F(" \t"));
    Serial.print(direction[0]);
    Serial.print(direction[1]);
    Serial.print(direction[2]);
    Serial.print(F("°"));
    Serial.println();

    serialTimer = millis();
  }
  //*****************************************************************************
}
