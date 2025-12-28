markdown
# Medical Sensors Library for ESP32-P4

A comprehensive Arduino library for reading multiple medical sensors on ESP32-P4 microcontroller.

## Supported Sensors
- BME280 (Environmental: Temperature, Humidity, Pressure)
- QMC5883L (GY-271 Magnetometer/Compass)
- MAX30205 (Precision Body Temperature)
- MAX30102 (Heart Rate and Blood Oxygen/SpO2)

## Features
- Unified interface for all sensors
- Automatic sensor detection and initialization
- Real-time heart rate and SpO2 monitoring
- Environmental data logging
- Built-in calibration and debugging tools

## Wiring Diagram
ESP32-P4 BME280 QMC5883L MAX30205 MAX30102
3.3V --------- VCC ------ VCC ------ VCC ------ VIN
GND --------- GND ------ GND ------ GND ------ GND
GPIO28 --------- SDA ------ SDA ------ SDA ------ SDA
GPIO32 --------- SCL ------ SCL ------ SCL ------ SCL

text

## I2C Addresses
- BME280: 0x76
- QMC5883L: 0x0D
- MAX30205: 0x48
- MAX30102: 0x57

## Installation

1. Clone this repository or download the source files
2. Place `SensorSystem.h` and `SensorSystem.cpp` in your Arduino project folder
3. Include the library in your sketch: `#include "SensorSystem.h"`

## Dependencies
- Adafruit BME280 Library
- QMC5883LCompass Library
- Protocentral_MAX30205 Library
- SparkFun MAX3010x Library

## Quick Start Example

```cpp
#include "SensorSystem.h"

SensorSystem sensors(28, 32);  // Custom I2C pins

void setup() {
  Serial.begin(115200);
  sensors.begin();
}

void loop() {
  sensors.update();
  
  SensorSystem::SensorData data = sensors.getData();
  
  // Access sensor data
  float temp = data.bme280.temperature;
  int heartRate = data.max30102.heartRate;
  float bodyTemp = data.max30205.temperature;
  // ... etc
}
API Reference
Constructor
cpp
SensorSystem(uint8_t sdaPin = 28, uint8_t sclPin = 32);
Methods
bool begin() - Initialize all sensors

void update() - Update sensor readings

SensorData getData() - Get latest sensor data

void i2cScanner() - Scan I2C bus for devices

void testMAX30102Signal() - Test MAX30102 signal quality

void calibrateCompass() - Calibrate magnetometer

Troubleshooting
MAX30102 Issues
Ensure finger is placed firmly on sensor

Apply consistent pressure (not too light, not too hard)

Wait 30+ seconds for stable readings

Check IR values (should be > 50,000 for good detection)

MAX30205 Issues
Check I2C address (should be 0x48)

Verify wiring connections

Run debugMAX30205() function for diagnostics

License
MIT License - Free to use for personal and commercial projects.

Contributing
Feel free to submit issues and pull requests to improve the library.

text

### File 5: `.gitignore`
Create this file to ignore unnecessary files:

```gitignore
# Arduino
*.elf
*.hex
*.eep
*.bin
*.lst
*.lss
*.sym
*.map
*.tmp
*.d
*.o
*.su
*.ino.bak

# IDE
.vscode/
.idea/
*.swp
*.swo

# OS
.DS_Store
Thumbs.db

# Build output
build/
Step 3: Commit and Push to GitHub
After creating all the files in your repository folder, run these commands:

bash
# Check the status of your files
git status

# Add all files to staging
git add .

# Commit with a message
git commit -m "Add complete Medical Sensors library for ESP32-P4 with BME280, QMC5883L, MAX30205, MAX30102 support"

# Push to GitHub
git push origin main
If you get an error about the branch name, try:

bash
git push origin master
Or if you need to set upstream:

bash
git push -u origin main