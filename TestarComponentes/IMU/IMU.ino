#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define TS 5000 // microssegundos
unsigned long t = micros();

// MPU6050 mpu;
Adafruit_MPU6050 mpu;
float g_x_offset = -0.05 + 0.097;
sensors_event_t a, g, temp;

float g_x, a_pitch, theta;
float alpha = 0.98;
void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);

  delay(1000);
  mpu.getEvent(&a, &g, &temp);
  theta = atan2(a.acceleration.z, a.acceleration.y);
}

void loop() {
  if (micros() - t >= TS){
    t = micros();
    
    mpu.getEvent(&a, &g, &temp);
    g_x = -(g.gyro.x + g_x_offset);
    a_pitch = atan2(a.acceleration.z, a.acceleration.y);

    theta = alpha*(theta + g_x*(TS/1e6)) + (1-alpha)*a_pitch;
  
    Serial.print(" Theta: ");
    Serial.print(theta);
    Serial.println();
  }
}
