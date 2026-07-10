#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println("Starting Hacked Adafruit MPU6050...");

  // 1. Force the ESP32 to permanently change its default I2C pins
  Wire.setPins(21, 23); 
  Wire.begin(); 
  
  delay(100); // Give the sensor a moment to wake up

  // 2. Initialize Adafruit (Passing the custom Wire object)
  if (!mpu.begin(0x68, &Wire)) {
    Serial.println("ERROR: Failed to find MPU6050 chip!");
    while (1) { delay(10); } 
  }
  
  Serial.println("SUCCESS: Adafruit Library Accepted the Sensor!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print("Yaw Rate (Z-axis): ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  delay(100); 
}