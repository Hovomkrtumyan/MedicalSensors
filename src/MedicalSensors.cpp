#include "MedicalSensors.h"

// Constructor
MedicalSensors::MedicalSensors(uint8_t sdaPin, uint8_t sclPin) 
  : _sdaPin(sdaPin), _sclPin(sclPin) {
  // Initialize rate array
  for (int i = 0; i < RATE_SIZE; i++) {
    _rates[i] = 0;
  }
}

// Initialize all sensors
bool MedicalSensors::begin() {
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
  
  // Initialize default values
  if (_max30102Present) {
    _data.max30102.spo2 = 0;
    _data.max30102.spo2Valid = false;
  }
  
  Serial.printf("[MedicalSensors] Initialized: BME280=%d, Compass=%d, MAX30205=%d, MAX30102=%d\n",
               _bmePresent, _hmcPresent, _max30205Present, _max30102Present);
  
  return (_bmePresent || _hmcPresent || _max30205Present || _max30102Present);
}

// Update all sensor readings
void MedicalSensors::update() {
  unsigned long currentMillis = millis();
  
  // Update at regular intervals
  if (currentMillis - _lastUpdate >= _updateInterval) {
    _lastUpdate = currentMillis;
    
    if (_bmePresent) updateBME280();
    if (_hmcPresent) updateCompass();
    if (_max30205Present) updateMAX30205();
  }
  
  // MAX30102 needs more frequent updates
  if (_max30102Present) {
    updateMAX30102();
  }
}

// Get latest data
MedicalSensors::SensorData MedicalSensors::getData() {
  return _data;
}

// Initialize BME280
bool MedicalSensors::initBME280() {
  if (_bme.begin(0x76)) {
    _bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                     Adafruit_BME280::SAMPLING_X2,
                     Adafruit_BME280::SAMPLING_X16,
                     Adafruit_BME280::SAMPLING_X1,
                     Adafruit_BME280::FILTER_X16,
                     Adafruit_BME280::STANDBY_MS_0_5);
    return true;
  }
  
  // Try alternative address
  if (_bme.begin(0x77)) {
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
bool MedicalSensors::initQMC5883L() {
  _compass.init();
  delay(100);
  _compass.read();
  
  // Test reading
  int testX = _compass.getX();
  int testY = _compass.getY();
  int testZ = _compass.getZ();
  
  // Also check if sensor responds
  _compass.setSmoothing(10, true);
  
  return (testX != 0 || testY != 0 || testZ != 0);
}

// Initialize MAX30205
bool MedicalSensors::initMAX30205() {
  Wire.beginTransmission(0x48);
  Wire.write(0x01);  // CONFIG register
  Wire.write(0x00);  // Continuous conversion
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    // Try alternative address
    Wire.beginTransmission(0x49);
    Wire.write(0x01);
    Wire.write(0x00);
    error = Wire.endTransmission();
    if (error != 0) return false;
  }
  
  delay(100);
  
  // Test reading
  float temp;
  Wire.beginTransmission(0x48);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0) {
    // Try alternative address
    Wire.beginTransmission(0x49);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0) return false;
  }
  
  uint8_t address = (error == 0) ? 0x48 : 0x49;
  Wire.requestFrom(address, 2);
  if (Wire.available() >= 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    uint16_t tempRaw = (msb << 8) | lsb;
    temp = tempRaw * 0.00390625;
    return (temp >= 15.0 && temp <= 50.0);
  }
  
  return false;
}

// Initialize MAX30102
bool MedicalSensors::initMAX30102() {
  if (!_particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    return false;
  }
  
  // Configure sensor
  byte ledBrightness = 0x1F; // Options: 0=Off to 255=50mA
  byte sampleAverage = 4;    // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;         // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 400;     // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;     // Options: 69, 118, 215, 411
  int adcRange = 4096;      // Options: 2048, 4096, 8192, 16384
  
  _particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  _particleSensor.enableDIETEMPRDY();
  _particleSensor.enableAFULL();
  _particleSensor.clearFIFO();
  
  return true;
}

// Update BME280 readings
void MedicalSensors::updateBME280() {
  if (!_bmePresent) return;
  
  _data.bme280.temperature = _bme.readTemperature();
  _data.bme280.pressure = _bme.readPressure() / 100.0F;
  _data.bme280.humidity = _bme.readHumidity();
  _data.bme280.altitude = _bme.readAltitude(1013.25);
}

// Update compass readings
void MedicalSensors::updateCompass() {
  if (!_hmcPresent) return;
  
  _compass.read();
  _data.compass.x = _compass.getX();
  _data.compass.y = _compass.getY();
  _data.compass.z = _compass.getZ();
  
  float heading = _compass.getAzimuth();
  
  // Normalize heading
  if (heading < 0) heading += 360;
  if (heading >= 360) heading -= 360;
  
  _data.compass.heading = heading;
  
  // Get direction
  char dir[4];
  _compass.getDirection(dir, heading);
  copyString(_data.compass.direction, dir, sizeof(_data.compass.direction));
}

// Update MAX30205 readings
void MedicalSensors::updateMAX30205() {
  if (!_max30205Present) return;
  
  Wire.beginTransmission(0x48);
  Wire.write(0x00);  // TEMP register
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom(0x48, 2);
    if (Wire.available() >= 2) {
      uint8_t msb = Wire.read();
      uint8_t lsb = Wire.read();
      
      uint16_t tempRaw = (msb << 8) | lsb;
      float temperature = tempRaw * 0.00390625;
      
      _data.max30205.temperature = temperature;
      
      // Determine fever status
      const char* status;
      if (temperature >= 39.0) {
        status = "HIGH FEVER";
      } else if (temperature >= 37.5) {
        status = "LOW FEVER";
      } else if (temperature >= 36.0) {
        status = "NORMAL";
      } else if (temperature >= 35.0) {
        status = "LOW";
      } else {
        status = "HYPOTHERMIA";
      }
      copyString(_data.max30205.feverStatus, status, sizeof(_data.max30205.feverStatus));
    }
  }
}

// Update MAX30102 readings
void MedicalSensors::updateMAX30102() {
  if (!_max30102Present) return;
  
  // Check if sensor data is available
  if (_particleSensor.available()) {
    // Read from FIFO
    long irValue = _particleSensor.getFIFOIR();
    long redValue = _particleSensor.getFIRED();
    
    _particleSensor.nextSample();
    
    _data.max30102.irValue = irValue;
    _data.max30102.redValue = redValue;
    _data.max30102.fingerDetected = (irValue > 50000);
    
    // Signal quality
    const char* quality;
    if (irValue < 5000) {
      quality = "NO SIGNAL";
    } else if (irValue < 20000) {
      quality = "WEAK";
    } else if (irValue < 50000) {
      quality = "FAIR";
    } else if (irValue < 100000) {
      quality = "GOOD";
    } else {
      quality = "EXCELLENT";
    }
    copyString(_data.max30102.signalQuality, quality, sizeof(_data.max30102.signalQuality));
    
    // Calculate heart rate if finger detected
    if (_data.max30102.fingerDetected) {
      calculateHeartRate();
      _data.max30102.heartRate = _beatAvg;
    } else {
      _data.max30102.heartRate = 0;
      _beatAvg = 0;
    }
    
    // Sensor temperature
    _data.max30102.sensorTemp = _particleSensor.readTemperature();
  }
}

// Calculate heart rate
void MedicalSensors::calculateHeartRate() {
  long irValue = _particleSensor.getIR();
  
  if (checkForBeat(irValue)) {
    unsigned long delta = millis() - _lastBeat;
    _lastBeat = millis();
    
    _beatsPerMinute = 60 / (delta / 1000.0);
    
    if (_beatsPerMinute < 255 && _beatsPerMinute > 20) {
      _rates[_rateSpot++] = (int)_beatsPerMinute;
      _rateSpot %= RATE_SIZE;
      
      // Calculate average
      _beatAvg = 0;
      for (int i = 0; i < RATE_SIZE; i++) {
        _beatAvg += _rates[i];
      }
      _beatAvg /= RATE_SIZE;
    }
  }
}

// I2C Scanner
void MedicalSensors::i2cScanner() {
  Serial.println("\n--- I2C Scanner ---");
  byte error, address;
  int nDevices = 0;
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Device found at 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      
      // Identify common devices
      if (address == 0x0D) Serial.print(" (QMC5883L)");
      else if (address == 0x1A) Serial.print(" (QMC5883L Alt)");
      else if (address == 0x48) Serial.print(" (MAX30205)");
      else if (address == 0x49) Serial.print(" (MAX30205 Alt)");
      else if (address == 0x57) Serial.print(" (MAX30102/MAX30105)");
      else if (address == 0x76) Serial.print(" (BME280)");
      else if (address == 0x77) Serial.print(" (BME280 Alt)");
      
      Serial.println();
      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  Serial.printf("Found %d device(s)\n\n", nDevices);
}