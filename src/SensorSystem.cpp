#include "SensorSystem.h"

// Constructor
SensorSystem::SensorSystem(uint8_t sdaPin, uint8_t sclPin) 
  : _sdaPin(sdaPin), _sclPin(sclPin) {
}

// Initialize all sensors
bool SensorSystem::begin() {
  // Initialize I2C
  Wire.begin(_sdaPin, _sclPin);
  Wire.setClock(400000);
  delay(100);
  
  // Initialize individual sensors
  _bmePresent = initBME280();
  _hmcPresent = initQMC5883L();
  _max30205Present = initMAX30205();
  _max30102Present = initMAX30102();
  
  // Initialize data structure
  _data.bme280.present = _bmePresent;
  _data.compass.present = _hmcPresent;
  _data.max30205.present = _max30205Present;
  _data.max30102.present = _max30102Present;
  
  return (_bmePresent || _hmcPresent || _max30205Present || _max30102Present);
}

// Update all sensor readings
void SensorSystem::update() {
  unsigned long currentMillis = millis();
  
  // Update at regular intervals
  if (currentMillis - _lastUpdate >= _updateInterval) {
    _lastUpdate = currentMillis;
    
    if (_bmePresent) updateBME280();
    if (_hmcPresent) updateCompass();
    if (_max30205Present) updateMAX30205();
  }
  
  // Always update MAX30102 (needs frequent updates)
  if (_max30102Present) updateMAX30102();
}

// Get latest sensor data
SensorSystem::SensorData SensorSystem::getData() {
  return _data;
}

// Initialize BME280
bool SensorSystem::initBME280() {
  if (_bme.begin(0x76)) {
    _bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                     Adafruit_BME280::SAMPLING_X2,
                     Adafruit_BME280::SAMPLING_X16,
                     Adafruit_BME280::SAMPLING_X1,
                     Adafruit_BME280::FILTER_X16,
                     Adafruit_BME280::STANDBY_MS_0_5);
    return true;
  }
  return false;
}

// Initialize QMC5883L
bool SensorSystem::initQMC5883L() {
  _compass.init();
  _compass.read();
  
  // Test reading
  int testX = _compass.getX();
  int testY = _compass.getY();
  int testZ = _compass.getZ();
  
  return (testX != 0 || testY != 0 || testZ != 0);
}

// Initialize MAX30205 using direct I2C
bool SensorSystem::initMAX30205() {
  Wire.beginTransmission(MAX30205_ADDR);
  Wire.write(MAX30205_CONFIG);
  Wire.write(0x00);  // Continuous conversion
  byte error = Wire.endTransmission();
  
  if (error != 0) return false;
  
  delay(100);
  
  // Test reading
  float temp;
  if (readMAX30205Direct(temp) && temp >= 15.0 && temp <= 50.0) {
    return true;
  }
  
  return false;
}

// Initialize MAX30102
bool SensorSystem::initMAX30102() {
  if (_particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    // Configure with optimal settings
    _particleSensor.setup(255, 1, 2, 100, 411, 4096);
    _particleSensor.enableDIETEMPRDY();
    _particleSensor.clearFIFO();
    
    // Initialize SPO2 buffers
    _bufferLength = 100;
    for (byte i = 0; i < _bufferLength; i++) {
      while (!_particleSensor.available())
        _particleSensor.check();
      
      _redBuffer[i] = _particleSensor.getRed();
      _irBuffer[i] = _particleSensor.getIR();
      _particleSensor.nextSample();
    }
    
    // Initial SPO2 calculation
    maxim_heart_rate_and_oxygen_saturation(_irBuffer, _bufferLength, _redBuffer, 
                                          &_spo2, &_validSPO2, &_heartRate, &_validHeartRate);
    
    return true;
  }
  return false;
}

// Update BME280 readings
void SensorSystem::updateBME280() {
  _data.bme280.temperature = _bme.readTemperature();
  _data.bme280.pressure = _bme.readPressure() / 100.0F;
  _data.bme280.humidity = _bme.readHumidity();
  _data.bme280.altitude = _bme.readAltitude(1013.25);
}

// Update compass readings
void SensorSystem::updateCompass() {
  _compass.read();
  _compassX = _compass.getX();
  _compassY = _compass.getY();
  _compassZ = _compass.getZ();
  _heading = _compass.getAzimuth();
  
  // Store in data structure
  _data.compass.x = _compassX;
  _data.compass.y = _compassY;
  _data.compass.z = _compassZ;
  
  // Normalize heading
  float normalizedHeading = _heading;
  if (normalizedHeading < 0) normalizedHeading += 360;
  if (normalizedHeading >= 360) normalizedHeading -= 360;
  
  _data.compass.heading = normalizedHeading;
  
  // Get direction
  _compass.getDirection(_directionArray, normalizedHeading);
  strncpy(_data.compass.direction, _directionArray, sizeof(_data.compass.direction));
}

// Update MAX30205 readings
void SensorSystem::updateMAX30205() {
  float temperature;
  if (readMAX30205Direct(temperature) && temperature >= 15.0 && temperature <= 50.0) {
    _data.max30205.temperature = temperature;
    
    // Determine fever status
    if (temperature >= 39.0) {
      _data.max30205.feverStatus = "HIGH FEVER";
    } else if (temperature >= 37.5) {
      _data.max30205.feverStatus = "LOW FEVER";
    } else if (temperature >= 36.0) {
      _data.max30205.feverStatus = "NORMAL";
    } else {
      _data.max30205.feverStatus = "BELOW NORMAL";
    }
  } else {
    _data.max30205.temperature = -100.0;
    _data.max30205.feverStatus = "INVALID";
  }
}

// Update MAX30102 readings
void SensorSystem::updateMAX30102() {
  long irValue = _particleSensor.getIR();
  long redValue = _particleSensor.getRed();
  
  // Store raw values
  _data.max30102.irValue = irValue;
  _data.max30102.redValue = redValue;
  
  // Finger detection
  _data.max30102.fingerDetected = (irValue > 50000);
  
  // Signal quality
  if (irValue < 5000) {
    _data.max30102.signalQuality = "NO SIGNAL";
  } else if (irValue < 20000) {
    _data.max30102.signalQuality = "WEAK";
  } else if (irValue < 50000) {
    _data.max30102.signalQuality = "FAIR";
  } else if (irValue < 100000) {
    _data.max30102.signalQuality = "GOOD";
  } else {
    _data.max30102.signalQuality = "EXCELLENT";
  }
  
  // Heart rate detection (simplified peak detection)
  if (_data.max30102.fingerDetected) {
    static long lastIR = 0;
    static bool wasRising = false;
    static unsigned long lastPeakTime = 0;
    
    bool isRising = (irValue > lastIR);
    
    if (wasRising && !isRising) {  // Peak detected
      unsigned long now = millis();
      if (lastPeakTime > 0) {
        unsigned long interval = now - lastPeakTime;
        if (interval > 300 && interval < 1500) {  // 40-200 BPM
          _beatsPerMinute = 60000.0 / interval;
          
          if (_beatsPerMinute >= 40 && _beatsPerMinute <= 200) {
            _rates[_rateSpot++] = (byte)_beatsPerMinute;
            _rateSpot %= RATE_SIZE;
            
            _beatAvg = 0;
            for (byte x = 0; x < RATE_SIZE; x++)
              _beatAvg += _rates[x];
            _beatAvg /= RATE_SIZE;
            
            _data.max30102.heartRate = _beatAvg;
          }
        }
      }
      lastPeakTime = now;
    }
    
    wasRising = isRising;
    lastIR = irValue;
  } else {
    _data.max30102.heartRate = 0;
    // Reset heart rate variables
    _beatAvg = 0;
    for (byte x = 0; x < RATE_SIZE; x++) _rates[x] = 0;
    _rateSpot = 0;
  }
  
  // SPO2 calculation
  if (_particleSensor.available() && _data.max30102.fingerDetected) {
    // Update buffers
    for (byte i = 0; i < _bufferLength - 1; i++) {
      _redBuffer[i] = _redBuffer[i + 1];
      _irBuffer[i] = _irBuffer[i + 1];
    }
    
    _redBuffer[_bufferLength - 1] = _particleSensor.getRed();
    _irBuffer[_bufferLength - 1] = _particleSensor.getIR();
    _particleSensor.nextSample();
    
    // Calculate SPO2 every 3 seconds
    static unsigned long lastSPO2Calc = 0;
    static int spo2Attempts = 0;
    
    if (millis() - lastSPO2Calc > 3000) {
      lastSPO2Calc = millis();
      
      // Reset before calculation
      _spo2 = 0;
      _validSPO2 = 0;
      
      maxim_heart_rate_and_oxygen_saturation(_irBuffer, _bufferLength, _redBuffer, 
                                            &_spo2, &_validSPO2, &_heartRate, &_validHeartRate);
      
      if (_validSPO2 == 1 && _spo2 >= 70 && _spo2 <= 100) {
        _data.max30102.spo2 = _spo2;
        _data.max30102.spo2Valid = true;
        spo2Attempts = 0;
      } else {
        _data.max30102.spo2Valid = false;
        spo2Attempts++;
        
        if (spo2Attempts > 5) {
          resetSPO2Calculation();
          spo2Attempts = 0;
        }
      }
    }
  }
  
  // Sensor temperature
  float sensorTemp = _particleSensor.readTemperature();
  if (sensorTemp > 0) {
    _data.max30102.sensorTemp = sensorTemp;
  }
}

// Direct MAX30205 temperature reading
bool SensorSystem::readMAX30205Direct(float &temperature) {
  Wire.beginTransmission(MAX30205_ADDR);
  Wire.write(MAX30205_TEMP);
  if (Wire.endTransmission(false) != 0) return false;
  
  Wire.requestFrom(MAX30205_ADDR, 2);
  if (Wire.available() >= 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    
    uint16_t tempRaw = (msb << 8) | lsb;
    temperature = tempRaw * 0.00390625;  // 0.00390625°C per LSB
    return true;
  }
  return false;
}

// Reset SPO2 calculation
void SensorSystem::resetSPO2Calculation() {
  for (byte i = 0; i < _bufferLength; i++) {
    while (!_particleSensor.available())
      _particleSensor.check();
    
    _redBuffer[i] = _particleSensor.getRed();
    _irBuffer[i] = _particleSensor.getIR();
    _particleSensor.nextSample();
  }
  
  _spo2 = 0;
  _validSPO2 = 0;
}

// I2C Scanner
void SensorSystem::i2cScanner() {
  Serial.println("\n--- I2C Scanner ---");
  byte error, address;
  int nDevices = 0;
  
  Wire.setClock(100000);  // Standard speed for scanning
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      if (address == 0x0D) Serial.print(" (QMC5883L)");
      else if (address == 0x48) Serial.print(" (MAX30205)");
      else if (address == 0x57) Serial.print(" (MAX30102)");
      else if (address == 0x76) Serial.print(" (BME280)");
      
      Serial.println();
      nDevices++;
    }
  }
  
  Serial.printf("Found %d device(s)\n", nDevices);
  Wire.setClock(400000);  // Back to fast speed
}

// Test MAX30102 signal
void SensorSystem::testMAX30102Signal() {
  if (!_max30102Present) {
    Serial.println("MAX30102 not initialized!");
    return;
  }
  
  Serial.println("\n--- MAX30102 Signal Test ---");
  Serial.println("Place finger on sensor, then remove it");
  Serial.println("Testing for 10 seconds...");
  
  unsigned long startTime = millis();
  long maxIR = 0;
  long minIR = 1000000;
  int samples = 0;
  
  while (millis() - startTime < 10000) {
    long irValue = _particleSensor.getIR();
    long redValue = _particleSensor.getRed();
    
    if (irValue > maxIR) maxIR = irValue;
    if (irValue < minIR) minIR = irValue;
    samples++;
    
    if (samples % 10 == 0) {
      Serial.printf("IR: %ld  Red: %ld\n", irValue, redValue);
    }
    
    delay(100);
  }
  
  Serial.println("\n--- Results ---");
  Serial.printf("Samples: %d\n", samples);
  Serial.printf("Max IR: %ld\n", maxIR);
  Serial.printf("Min IR: %ld\n", minIR);
  Serial.printf("Avg IR: %ld\n", (maxIR + minIR) / 2);
  
  if (maxIR > 50000) {
    Serial.println("✓ Finger detection works");
  } else {
    Serial.println("✗ Finger detection poor - check placement");
  }
}

// Calibrate compass
void SensorSystem::calibrateCompass() {
  if (!_hmcPresent) {
    Serial.println("Compass not initialized!");
    return;
  }
  
  Serial.println("\n--- Compass Calibration ---");
  Serial.println("Rotate sensor slowly in all directions for 20 seconds...");
  
  unsigned long startTime = millis();
  int minX = 0, maxX = 0;
  int minY = 0, maxY = 0;
  int minZ = 0, maxZ = 0;
  
  while (millis() - startTime < 20000) {
    _compass.read();
    int x = _compass.getX();
    int y = _compass.getY();
    int z = _compass.getZ();
    
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
    if (z < minZ) minZ = z;
    if (z > maxZ) maxZ = z;
    
    delay(50);
    if (millis() % 1000 < 50) Serial.print(".");
  }
  
  Serial.println("\n\nCalibration complete!");
  Serial.println("Min/Max values:");
  Serial.printf("X: %d to %d\n", minX, maxX);
  Serial.printf("Y: %d to %d\n", minY, maxY);
  Serial.printf("Z: %d to %d\n", minZ, maxZ);
}

// Debug MAX30205
void SensorSystem::debugMAX30205() {
  Serial.println("\n--- MAX30205 Debug ---");
  
  Wire.setClock(100000);
  
  Wire.beginTransmission(MAX30205_ADDR);
  byte error = Wire.endTransmission();
  Serial.printf("I2C error code: %d\n", error);
  
  if (error == 0) {
    Serial.println("Device responds at 0x48");
    
    for (int i = 0; i < 3; i++) {
      float temp;
      if (readMAX30205Direct(temp)) {
        Serial.printf("Read %d: %.3f °C\n", i + 1, temp);
      } else {
        Serial.printf("Read %d: FAILED\n", i + 1);
      }
      delay(100);
    }
  } else {
    Serial.println("Device not responding at 0x48");
  }
  
  Wire.setClock(400000);
}