#ifndef SENSOR_SYSTEM_H
#define SENSOR_SYSTEM_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Protocentral_MAX30205.h"
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include <QMC5883LCompass.h>

class SensorSystem {
public:
  // Constructor
  SensorSystem(uint8_t sdaPin = 28, uint8_t sclPin = 32);
  
  // Initialization
  bool begin();
  
  // Update sensor readings
  void update();
  
  // Get sensor data
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
      String feverStatus;
    } max30205;
    
    // MAX30102
    struct {
      int heartRate;
      int spo2;
      long irValue;
      long redValue;
      float sensorTemp;
      bool fingerDetected;
      String signalQuality;
      bool present;
      bool spo2Valid;
    } max30102;
  };
  
  // Get latest data
  SensorData getData();
  
  // Get individual sensor status
  bool isBME280Present() { return _bmePresent; }
  bool isCompassPresent() { return _hmcPresent; }
  bool isMAX30205Present() { return _max30205Present; }
  bool isMAX30102Present() { return _max30102Present; }
  
  // Utility functions
  void i2cScanner();
  void testMAX30102Signal();
  void calibrateCompass();
  void debugMAX30205();
  
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
  
  // MAX30102 variables
  static const byte RATE_SIZE = 4;
  byte _rates[RATE_SIZE];
  byte _rateSpot = 0;
  long _lastBeat = 0;
  float _beatsPerMinute;
  int _beatAvg;
  
  uint32_t _irBuffer[100];
  uint32_t _redBuffer[100];
  int32_t _bufferLength;
  int32_t _spo2;
  int8_t _validSPO2;
  int32_t _heartRate;
  int8_t _validHeartRate;
  
  // Compass variables
  int _compassX, _compassY, _compassZ;
  float _heading;
  char _directionArray[4];
  
  // Timing
  unsigned long _lastUpdate = 0;
  const unsigned long _updateInterval = 2000;
  
  // Data structure
  SensorData _data;
  
  // Private methods
  bool initBME280();
  bool initQMC5883L();
  bool initMAX30205();
  bool initMAX30102();
  
  void updateBME280();
  void updateCompass();
  void updateMAX30205();
  void updateMAX30102();
  
  bool readMAX30205Direct(float &temperature);
  void resetSPO2Calculation();
  
  // MAX30205 registers
  static const uint8_t MAX30205_ADDR = 0x48;
  static const uint8_t MAX30205_TEMP = 0x00;
  static const uint8_t MAX30205_CONFIG = 0x01;
};

#endif