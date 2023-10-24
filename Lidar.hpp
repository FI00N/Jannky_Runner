#include <Arduino.h>
#include <VL6180X.h>
#include <VL53L0X.h>
#include <Wire.h>

namespace mtrn3100 {

class Lidar {
public:
    Lidar(const int address, const int enablePin, VL6180X sensor) 
    : address(address), enablePin(enablePin), sensor(sensor) {
    }

    void lidarSetup() {
      pinMode(enablePin, OUTPUT);
      // Reset all connected sensors
      digitalWrite(enablePin, LOW);
      delay(1000);

      digitalWrite(enablePin, HIGH);
      delay(50);
      sensor.init();
      sensor.configureDefault();
      sensor.setAddress(address);  // Set the new address.
      Serial.print("sensor I2C address: 0x");
      Serial.println(sensor.readReg(0x212), HEX);
      delay(100);
    }

    float readDistance() {
      return sensor.readRangeSingleMillimeters();
    }

private:
    VL6180X sensor;
    const int address;
    const int enablePin;
};

class longRangeLidar {
public:
    longRangeLidar(const int address, const int enablePin, VL53L0X sensor) 
    : address(address), enablePin(enablePin), sensor(sensor) {
    }

    void setupFrontLidar() {
        delay(100);
        pinMode(enablePin, OUTPUT);
        digitalWrite(enablePin, LOW);
        delay(1000);
        Serial.println("Starting front sensor...");
        digitalWrite(enablePin, HIGH);
        if (!sensor.init()) {
            Serial.println("Failed to detect and initialize sensor!");
            while (1) {}
        }
        Serial.println("Success!");
        delay(100);
    }

    float readDistance() {
        return sensor.readRangeSingleMillimeters();
    }

private:
    VL53L0X sensor;
    const int address;
    const int enablePin;
};
}  // namespace mtrn3100