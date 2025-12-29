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