#ifndef MEDICAL_SENSORS_H
#define MEDICAL_SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Protocentral_MAX30205.h"
#include "MAX30105.h"
#include <QMC5883LCompass.h>

class MedicalSensors {
public:
  // Constructor
  MedicalSensors(uint8_t sdaPin = 21, uint8_t sclPin = 22); // Changed to common ESP32 defaults
  
  // Initialization
  bool begin();
  
  // Update sensor readings
  void update();
  
  // Sensor data structure
  struct SensorData {
    // BME280
    struct {
      float temperature;
      float pressure;
      float humidity;
      float altitude;
      bool present;
    } bme280;
    
    // QMC5883L
    struct {
      int16_t x;
      int16_t y;
      int16_t z;
      float heading;
      char direction[4];
      bool present;
    } compass;
    
    // MAX30205
    struct {
      float temperature;
      bool present;
      char feverStatus[20];
    } max30205;
    
    // MAX30102
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
  };
  
  // Get latest data
  SensorData getData();
  
  // Check sensor status
  bool isBME280Present() const { return _bmePresent; }
  bool isCompassPresent() const { return _hmcPresent; }
  bool isMAX30205Present() const { return _max30205Present; }
  bool isMAX30102Present() const { return _max30102Present; }
  
  // I2C Scanner
  void i2cScanner();
  
  // Configuration
  void setUpdateInterval(unsigned long interval) { _updateInterval = interval; }
  unsigned long getUpdateInterval() const { return _updateInterval; }

private:
  // I2C pins
  uint8_t _sdaPin;
  uint8_t _sclPin;
  
  // Sensor objects
  Adafruit_BME280 _bme;
  MAX30205 _tempSensor;
  MAX30105 _particleSensor;
  QMC5883LCompass _compass;
  
  // Sensor status
  bool _bmePresent = false;
  bool _hmcPresent = false;
  bool _max30205Present = false;
  bool _max30102Present = false;
  
  // Data structure
  SensorData _data;
  
  // Timing
  unsigned long _lastUpdate = 0;
  unsigned long _updateInterval = 2000;
  
  // Private methods
  bool initBME280();
  bool initQMC5883L();
  bool initMAX30205();
  bool initMAX30102();
  
  void updateBME280();
  void updateCompass();
  void updateMAX30205();
  void updateMAX30102();
  
  // Heart rate calculation
  void calculateHeartRate();
  
  // Helper to copy strings safely
  void copyString(char* dest, const char* src, size_t destSize) {
    if (src && dest && destSize > 0) {
      strncpy(dest, src, destSize - 1);
      dest[destSize - 1] = '\0';
    }
  }
  
  // Heart rate calculation variables
  static const int RATE_SIZE = 4;
  int _rates[RATE_SIZE];
  int _rateSpot = 0;
  unsigned long _lastBeat = 0;
  float _beatsPerMinute = 0;
  int _beatAvg = 0;
};

#endif