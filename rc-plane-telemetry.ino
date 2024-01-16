/* *************************************************************************** *
 * Project Title: RC plane telemetry
 * 
 * Description: A small Arduino project that uses various sensors to gather
 *              data and sends it back to the user radio control unit.
 *
 * Licence:   GPL 3.0       (details in project file "LICENCE")
 * Version:   1.0.0         (details in project file "README.md")
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
static const byte motorPoles = 14;   // number of magnets on motor
static const int rpmDelay = 250;     // delay timer for RPM reporting
static const int serialDelay = 500;  // timer for serial console reporting
static bool iBusDebug = false;       // there can only be one Serial, either for USB port (true) or iBus (false)

// Define variables
int inputVoltage = 0;           // battery voltage
int baseAltitude = 0;           // altitude at the start
int altitude = 0;               // altitude - from barometer
int azimuth = 0;                // azimuth (in degress) - from compass
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
  //*****************************************************************************
  if (iBusDebug == true) {       // output is send through USB port to the Serial Monitor
    Serial.begin(9600);          // debugging
    Serial.println();            // start fresh on a new line
  } else {                       // output is send through RX/TX ports to the receiver
    iBusSensor.begin(Serial);    // iBusSensor.begin(Serial, IBUSBM_NOTIMER);
    iBusSensor.addSensor(0x03);  // External Voltage	[V]   ()
    iBusSensor.addSensor(0x02);  // RPM							  [RPM] ()
    iBusSensor.addSensor(0xfd);  // Relative altitude	[m]   ()
    iBusSensor.addSensor(0xfd);  // Azimuth						[°]   ()
    iBusSensor.addSensor(0x01);  // Temperature				[°C]  (400 + temp * 10)
    iBusSensor.addSensor(0x41);  // Pressure					[hPa] ()
  }
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
  sensorHMC5883L.setSmoothing(10, true);
  sensorHMC5883L.setCalibrationOffsets(-220.00, 386.00, 166.00);
  sensorHMC5883L.setCalibrationScales(1.06, 0.84, 1.16);
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
  altitude = int(sensorBMP280.readAltitude() - baseAltitude);  // value is the difference between base and current altitude
  //*****************************************************************************

  // Get data from compass
  //*****************************************************************************
  sensorHMC5883L.read();
  azimuth = sensorHMC5883L.getAzimuth();  // read the sensor
  azimuth = 180 - azimuth;                // sensor is mounted upside down
  azimuth = 90 + azimuth;                 // sensor is mounted at 90° angle
  if (azimuth > 360) {                    // when correcting the angle we get interval 90 - 450
    azimuth = azimuth - 360;              // angle over 360° is actually from 0° - 90°
  }
  //*****************************************************************************

  // Get data from RPM sensor
  //*****************************************************************************
  if (millis() - rpmTimer > rpmDelay) {
    rpmValue = ((float(rpmCounter * 2 / motorPoles)) / (millis() - rpmTimer)) * 1000 * 60;
    rpmCounter = 0;
    rpmTimer = millis();
  }
  //*****************************************************************************

  // display sensor data
  //*****************************************************************************
  if (millis() - serialTimer >= serialDelay) {  // we use nonblocking delay not to overflow Serial
    if (iBusDebug == true) {                    // output is send through USB port to the Serial Monitor
      Serial.print(inputVoltage);
      Serial.print(F("\t"));
      Serial.print(int(rpmValue));
      Serial.print(F("\t"));
      Serial.print(altitude);
      Serial.print(F("\t"));
      Serial.print(azimuth);
      Serial.print(F("\t"));
      Serial.print(int(sensorBMP280.readTemperature() * 10) + 400);
      Serial.print(F("\t"));
      Serial.print(int(sensorBMP280.readPressure() / 100));
      Serial.print(F("\t"));
      Serial.println();
    } else {                                                                               // output is send through RX/TX ports to the receiver
      iBusSensor.setSensorMeasurement(1, inputVoltage);                                    // External Voltage	  [V]   ()
      iBusSensor.setSensorMeasurement(2, int(rpmValue));                                   // RPM							  []    ()
      iBusSensor.setSensorMeasurement(3, altitude);                                        // Relative altitude	[m]   ()
      iBusSensor.setSensorMeasurement(4, azimuth);                                         // Azimuth						[°]   ()
      iBusSensor.setSensorMeasurement(5, int(sensorBMP280.readTemperature() * 10) + 400);  // Temperature				[°C]  (400 + temp * 10)
      iBusSensor.setSensorMeasurement(6, int(sensorBMP280.readPressure() / 100));          // Pressure					  [hPa] ()
      iBusSensor.loop();
    }
    serialTimer = millis();
  }
  //*****************************************************************************
}
