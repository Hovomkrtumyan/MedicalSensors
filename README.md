# MedicalSensors Library

Arduino library for medical sensors including BME280, QMC5883L, MAX30205, and MAX30102.

## Features

- **BME280**: Temperature, humidity, pressure, altitude
- **QMC5883L**: 3-axis magnetometer (compass)
- **MAX30205**: Medical-grade body temperature sensor
- **MAX30102**: Heart rate and pulse oximetry sensor

## Installation

### PlatformIO
Add to your `platformio.ini`:
```ini
lib_deps = 
    https://github.com/Hovomkrtumyan/MedicalSensors.git


    Arduino IDE
Download the library as ZIP

Sketch → Include Library → Add .ZIP Library

Select the downloaded ZIP file

Wiring
Sensor	SDA Pin	SCL Pin	VCC	GND
BME280	SDA	SCL	3.3V	GND
QMC5883L	SDA	SCL	3.3V	GND
MAX30205	SDA	SCL	3.3V	GND
MAX30102	SDA	SCL	3.3V	GND
Note: All sensors share the same I2C bus.

Quick Start
cpp
#include <MedicalSensors.h>

MedicalSensors sensors(21, 22);  // SDA, SCL pins

void setup() {
  Serial.begin(115200);
  
  if (!sensors.begin()) {
    Serial.println("Failed to initialize sensors!");
    while (1);
  }
  
  sensors.i2cScanner();
}

void loop() {
  sensors.update();
  
  MedicalSensors::SensorData data = sensors.getData();
  
  if (data.bme280.present) {
    Serial.printf("Temp: %.1f°C, Humidity: %.1f%%, Pressure: %.1fhPa\n",
                  data.bme280.temperature,
                  data.bme280.humidity,
                  data.bme280.pressure);
  }
  
  if (data.max30205.present) {
    Serial.printf("Body Temp: %.2f°C - %s\n",
                  data.max30205.temperature,
                  data.max30205.feverStatus);
  }
  
  if (data.max30102.present) {
    Serial.printf("Heart Rate: %d BPM, Finger: %s\n",
                  data.max30102.heartRate,
                  data.max30102.fingerDetected ? "YES" : "NO");
  }
  
  delay(1000);
}
API Reference
Constructor
cpp
MedicalSensors(uint8_t sdaPin = 21, uint8_t sclPin = 22);
Methods
bool begin(): Initialize all sensors

void update(): Update sensor readings

SensorData getData(): Get latest sensor data

void i2cScanner(): Scan I2C bus for devices

void setUpdateInterval(unsigned long interval): Set update frequency

bool isBME280Present(): Check if BME280 is connected

bool isMAX30102Present(): Check if MAX30102 is connected

bool isMAX30205Present(): Check if MAX30205 is connected

bool isCompassPresent(): Check if QMC5883L is connected

Data Structure
cpp
struct SensorData {
  struct {
    float temperature;
    float pressure;
    float humidity;
    float altitude;
    bool present;
  } bme280;
  
  struct {
    float temperature;
    bool present;
    char feverStatus[20];
  } max30205;
  
  struct {
    int heartRate;
    int spo2;
    long irValue;
    long redValue;
    float sensorTemp;
    bool fingerDetected;
    char signalQuality[20];
    bool present;
    bool spo2Valid;
  } max30102;
  
  struct {
    int16_t x, y, z;
    float heading;
    char direction[4];
    bool present;
  } compass;
};
Examples
Check the examples folder for:

BasicUsage - Basic sensor reading example

IndividualSensors - Using sensors individually

DataLogger - Logging data to SD card

Dependencies
Adafruit BME280 Library

MAX30205 Library

SparkFun MAX3010x Library

QMC5883LCompass Library

License
MIT License

Support
For issues and questions, please create an issue on GitHub.

text

## 6. Create Example Files

Create `examples/BasicUsage/BasicUsage.ino`:

```cpp
/*
  Basic Medical Sensors Example
  Demonstrates reading all medical sensors
*/

#include <MedicalSensors.h>

// Initialize with default I2C pins (21, 22 for ESP32)
MedicalSensors sensors;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== Medical Sensors Example ===");
  
  // Initialize sensors
  if (!sensors.begin()) {
    Serial.println("Failed to initialize sensors!");
    Serial.println("Please check connections and restart.");
    while (1);
  }
  
  Serial.println("Sensors initialized successfully!");
  
  // Scan I2C bus
  sensors.i2cScanner();
  
  Serial.println("\nStarting sensor readings...\n");
}

void loop() {
  // Update sensor readings
  sensors.update();
  
  // Get latest data
  MedicalSensors::SensorData data = sensors.getData();
  
  // Print separator
  Serial.println("----------------------------------------");
  
  // BME280 Data
  if (data.bme280.present) {
    Serial.println("BME280 (Environment):");
    Serial.printf("  Temperature: %.1f °C\n", data.bme280.temperature);
    Serial.printf("  Humidity:    %.1f %%\n", data.bme280.humidity);
    Serial.printf("  Pressure:    %.1f hPa\n", data.bme280.pressure);
    Serial.printf("  Altitude:    %.1f m\n", data.bme280.altitude);
  }
  
  // MAX30205 Data (Body Temperature)
  if (data.max30205.present) {
    Serial.println("\nMAX30205 (Body Temperature):");
    Serial.printf("  Temperature: %.2f °C\n", data.max30205.temperature);
    Serial.printf("  Status:      %s\n", data.max30205.feverStatus);
  }
  
  // MAX30102 Data (Heart Rate)
  if (data.max30102.present) {
    Serial.println("\nMAX30102 (Heart Rate):");
    Serial.printf("  Heart Rate:  %d BPM\n", data.max30102.heartRate);
    Serial.printf("  Finger:      %s\n", data.max30102.fingerDetected ? "Detected" : "Not Detected");
    Serial.printf("  Signal:      %s\n", data.max30102.signalQuality);
    Serial.printf("  IR Value:    %ld\n", data.max30102.irValue);
    Serial.printf("  Red Value:   %ld\n", data.max30102.redValue);
    Serial.printf("  Sensor Temp: %.1f °C\n", data.max30102.sensorTemp);
  }
  
  // Compass Data
  if (data.compass.present) {
    Serial.println("\nQMC5883L (Compass):");
    Serial.printf("  Heading:     %.1f°\n", data.compass.heading);
    Serial.printf("  Direction:   %s\n", data.compass.direction);
    Serial.printf("  X: %d, Y: %d, Z: %d\n", 
                  data.compass.x, data.compass.y, data.compass.z);
  }
  
  // Wait 2 seconds before next reading
  delay(2000);
}