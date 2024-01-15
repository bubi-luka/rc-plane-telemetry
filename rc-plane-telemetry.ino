/* *************************************************************************** *
 * Project Title: RC plane telemetry
 * 
 * Description: A small Arduino project that uses various sensors to gather
 *              data and sends it back to the user radio control unit.
 *
 * Licence: GPL 3.0       (details in project file "LICENCE")
 * Version: 0.6.0         (details in project file "README.md")
 * created: 25. 09. 2023
 * by:      Luka Oman
 * *************************************************************************** */

// Declare additional libraries
#include <IBusBM.h>           // Library for iBus telemetry
#include <Adafruit_BMP280.h>  // Library for BMP-280  sensor
#include <QMC5883LCompass.h>  // Library for *MC-5883L sensor
#include <MPU6050_light.h>    // Library for MPU-6050 sensor
#include <SdFat.h>            // Library for SD card

// Define Arduino pins
static const int voltagePin = A3;  // Analog 3 => for voltage data
static const int sdaPin = A4;      // Analog 4 => SDA pin for I2C
static const int sclPin = A5;      // Analog 5 => SCL pin for I2C

// Define new objects
IBusBM iBusSensor;               // iBus telecomunication for sensors
Adafruit_BMP280 sensorBMP280;    // pressure, temperature, altitude
QMC5883LCompass sensorHMC5883L;  // compass
MPU6050 sensorMPU6050(Wire);     // gyroscope, accelometer
SdFat sd;                        // SD card formated as FAT16
File logFile;                    // file that will contain log data

// Define static placeholders
#define fileBaseName "data-"  // define first part of file name
#define fileBaseExt ".log"    // define the file extension

// Define variables
int inputVoltage = 0;                                   // battery voltage
int baseAltitude = 0;                                   // altitude at the start
int altitude = 0;                                       // altitude - from barometer
int azimuth = 0;                                        // azimuth - from compass
char direction[3];                                      // direction - from compass
int axis_x = 0;                                         // X axis - from gyro
int axis_y = 0;                                         // Y axis - from gyro
int axis_z = 0;                                         // Z axis - from gyro
const uint8_t fileNameSize = sizeof(fileBaseName) - 1;  // position of numerals in file name
char fileName[] = fileBaseName "00" fileBaseExt;        // file name string

// Define timers
unsigned long previousSdTimer = 0;       // time of previous write to SD card
static const int intervalSdTimer = 500;  // interval at which we write data to the card

// Define custom functions

// Initialization function, run only once
void setup() {

  // Start communication protocols
  Serial.begin(9600);  // debugging
  Serial.println();    // start fresh on a new line

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

  // start HMC-5883L - compass
  //*****************************************************************************
  sensorHMC5883L.init();
  //*****************************************************************************

  // start MPU 6050 sensor - gyroscope
  //*****************************************************************************
  sensorMPU6050.begin();  // start sensor
  //sensorMPU6050.upsideDownMounting = true;  // MPU6050 is mounted upside-down
  delay(1000);                  // delay to stabilise the sensor
  sensorMPU6050.calcOffsets();  // calibration of gyrometer and accelerometer
  //*****************************************************************************

  // start SD card
  //*****************************************************************************
  while (!sd.begin()) {  // we try to start SD card
    delay(500);          // if we fail we wait set interval and try again
  }

  // we set new unique file name for each new run, we can create 100 files
  while (sd.exists(fileName)) {                  // if file name exist we increase file name by 1
    if (fileName[fileNameSize + 1] != '9') {     // if last numeral is not 9
      fileName[fileNameSize + 1]++;              // we increase the last numeral by 1
    } else if (fileName[fileNameSize] != '9') {  // if first numeral is not 9
      fileName[fileNameSize + 1] = '0';          // we set last numeral as zero
      fileName[fileNameSize]++;                  // and increase the first numeral by 1
    } else {                                     // if we have created 100 files
      fileName[fileNameSize] = '9';              // we set file name to the last possible place
      fileName[fileNameSize + 1] = '9';          // we set file name to the last possible place
    }
  }

  // we create a new file if not exists, else we append data to old file
  while (!logFile.open(fileName, O_WRITE | O_APPEND | O_CREAT)) {  //we try to open new file
    delay(500);                                                    // if we fail we wait set interval and try again
  }

  // print the header row
  logFile.print(F("Running\tDate\tTime\tX Coordinates\tY Coordinates\tAltitude\tTemperature\tPressure\tX axis\tY axis\tZ axis\tBattery\r\n"));
  logFile.flush();  // we write the data to the file
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

  // Get data from gyro
  //*****************************************************************************
  sensorMPU6050.update();
  axis_x = map(sensorMPU6050.getAngleX(), -90, 90, 0, 180);
  axis_y = map(sensorMPU6050.getAngleY(), -90, 90, 0, 180);
  axis_z = map(sensorMPU6050.getAngleZ(), -90, 90, 0, 359);
  //*****************************************************************************

  // Send gathered data to the remote control unit on the ground
  //*****************************************************************************
  iBusSensor.setSensorMeasurement(1, 655);                                // Temperature				[°C]  (400 + temp * 10)
  iBusSensor.setSensorMeasurement(2, inputVoltage);                       // External Voltage	  [V]   ()
  iBusSensor.setSensorMeasurement(3, sensorBMP280.readPressure() / 100);  // Pressure					  [hPa] ()
  iBusSensor.setSensorMeasurement(4, altitude);                           // Relative altitude	[m]   ()
  iBusSensor.setSensorMeasurement(5, axis_z);                             // Azimuth						[°]   ()
  iBusSensor.setSensorMeasurement(6, 17);                                 // Speed							[m/s] ()

  // send data to iBus
  iBusSensor.loop();
  //*****************************************************************************

  // Save current data to the SD card every set interval
  //*****************************************************************************
  if (millis() - previousSdTimer >= intervalSdTimer) {  // if the difference between previous time and current time is greater than set interval we write data to the card

    logFile.print(millis());
    logFile.print(F("\t"));
    logFile.print(F("Date"));
    logFile.print(F("\t"));
    logFile.print(F("Time"));
    logFile.print(F("\t"));
    logFile.print(F("North"));
    logFile.print(F("\t"));
    logFile.print(F("East"));
    logFile.print(F("\t"));
    logFile.print(altitude);
    logFile.print(F("\t"));
    logFile.print(sensorMPU6050.getTemp() - 20.5);
    logFile.print(F("\t"));
    logFile.print(sensorBMP280.readTemperature() - 5);
    logFile.print(F("\t"));
    logFile.print(sensorBMP280.readPressure() / 100);
    logFile.print(F("\t"));
    logFile.print(axis_x);
    logFile.print(F("\t"));
    logFile.print(axis_y);
    logFile.print(F("\t"));
    logFile.print(axis_z);
    logFile.print(F("\t"));
    logFile.print(inputVoltage);
    logFile.print(F("\r\n"));
    logFile.flush();

    Serial.print(millis());
    Serial.print(F("\t"));
    Serial.print(float(inputVoltage) / 100, 2);
    Serial.print(F(" V\t"));
    Serial.print(inVol);
    Serial.print(F(" V\t"));
    Serial.print(F("Date"));
    Serial.print(F("\t"));
    Serial.print(F("Time"));
    Serial.print(F("\t"));
    Serial.print(F("North"));
    Serial.print(F("\t"));
    Serial.print(F("East"));
    Serial.print(F("\t"));
    Serial.print(sensorMPU6050.getTemp());
    Serial.print(F("\t"));
    Serial.print(sensorBMP280.readTemperature());
    Serial.print(F(" °C\t"));
    Serial.print(sensorBMP280.readPressure() / 100);
    Serial.print(F(" hPa\t"));
    Serial.print(sensorBMP280.readAltitude());
    Serial.print(F(" m\t"));
    Serial.print(altitude);
    Serial.print(F(" m\t"));
    Serial.print(direction[0]);
    Serial.print(direction[1]);
    Serial.print(direction[2]);
    Serial.print(F(" \t"));
    Serial.print(azimuth);
    Serial.print(F(" \t"));
    Serial.print(axis_x);
    Serial.print(F("\t"));
    Serial.print(axis_y);
    Serial.print(F("\t"));
    Serial.print(axis_z);

    Serial.println();

    previousSdTimer = millis();
  }
  //*****************************************************************************
}
