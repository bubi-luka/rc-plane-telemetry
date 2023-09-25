# RC Plane Telemetry
**An Arduino project that uses various sensors to gather data and sends it back to the user radio control unit.**

## Description

### Materials
- Arduino Nano board
- radio receiver TGY-IA6B or similar
- wires
- diode 1N4148
- resistors (1x 10kΩ and 1x 100kΩ)

### Used libraries
- [iBusBM](https://github.com/bmellink/IBusBM/)
**!! Thank you all for your work !!**

## Installation and Usage
1. Clone the repository
2. Connect sensors to the Arduino
3. Compile and upload the code to the Arduino
4. Connect the receiver wires for the iBUS (only after uploading the code)
5. Wait for all the sensors start and GPS connects
6. Fly and enjoy

## Custumizations
It is possible to define up to 10 sensors. The available codes are:
* iBusSensor.addSensor(0x00); // Internal Voltage [V]   ()
* iBusSensor.addSensor(0x01); // Temperature      [°C]  (400 + temp * 10)
* iBusSensor.addSensor(0x02); // Motor rotations  [RPM] ()
* iBusSensor.addSensor(0x03); // External Voltage [V]   ()
* iBusSensor.addSensor(0x41); // Pressure         [hPa] ()
* iBusSensor.addSensor(0xfd); // Servo            [?]   ()

## Roadmap
### 0.3.0
- add altitude sensor

### 0.4.0
- add gyroscope sensor

### 0.5.0
- add compass sensor

### 0.6.0
- add SD card reader/writer module to log data

### 0.7.0
- add GPS sensor

## Change Log
### 0.2.0
- add external battery voltage sensor

### 0.1.0
- add iBUS communication path to send data from Arduino to RC transmitter module
