// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

const int buttonpin = 2;

float angX = 0.0;
float angY = 0.0;
float angZ = 0.0;

float vx = 0.0;
float vy = 0.0;
float vz = 0.0;

float px = 0.0;
float py = 0.0;
float pz = 0.0;

Adafruit_MPU6050 mpu;

void setup(void) {
  pinMode(buttonpin, INPUT);
  
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("");
  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accx = a.acceleration.x;
  float accy = a.acceleration.y;
  float accz = a.acceleration.z;
  float roll = g.gyro.x;
  float pitch = g.gyro.y;
  float yaw = g.gyro.z;

  // Calculate angle relative to starting angle
  angX += roll * 0.0375;
  angY += pitch * 0.0375;
  angZ += yaw * 0.0375;

  // Remove gravity from acceleration
  float ax = accx, ay = accy, az = 0;
  accx -= ax * cos(yaw) - ay * sin(yaw);
  accy -= ax * sin(yaw) - ay * cos(yaw);
  
  ax = accx; az = accz;
  accx -=  ax * cos(pitch) + az * sin(pitch);
  accz -= -ax * sin(pitch) + az * cos(pitch);
  
  ay = accy, az = accz;
  accy -=  ay * cos(roll) - az * sin(roll);
  accz -=  ay * sin(roll) + az * cos(roll);

  // Calculate current position
//  if (abs(accx) >= 0.75) {vx += accx * 0.0375;}
//  if (abs(accy) >= 0.75) {vy += accy * 0.0375;}
//  if (abs(accz) >= 0.75) {vz += accz * 0.0375;}
  
  if (abs(accx) >= 1.25) {
//    x += vx * 0.0375 + 0.5 * accx * 0.0375 * 0.0375;
    px += 0.5 * accx * 0.0375 * 0.0375;
  }
  if (abs(accy) >= 1.25) {
//    y += vy * 0.0375 + 0.5 * accy * 0.0375 * 0.0375;
    py += 0.5 * accy * 0.0375 * 0.0375;
  }
  if (abs(accz) >= 1.25) {
//    y += vy * 0.0375 + 0.5 * accy * 0.0375 * 0.0375;
    pz += 0.5 * accz * 0.0375 * 0.0375;
  }

  // Print out the values
  if (digitalRead(buttonpin) == HIGH) {Serial.print("1,");} 
  else {Serial.print("0,");}
  
  Serial.print(px);
  Serial.print(",");
  Serial.print(py);
  Serial.print(",");
  Serial.print(pz);
  Serial.print(",");
//  Serial.print(vx);
//  Serial.print(",");
//  Serial.print(vy);
//  Serial.print(",");
//  Serial.print(accx);
//  Serial.print(",");
//  Serial.print(accy);
//  Serial.print(",");
//  Serial.print(accz);
//  Serial.print(",");
  Serial.print(angX);
  Serial.print(",");
  Serial.print(angY);
  Serial.print(",");
  Serial.print(angZ);
  Serial.println("");

  delay(30);
}
