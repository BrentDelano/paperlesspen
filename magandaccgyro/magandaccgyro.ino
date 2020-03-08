#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
#include <Adafruit_MPU6050.h>

// i2c
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
Adafruit_MPU6050 mpu;

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5
// You can also use software SPI
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_SCK, LSM9DS1_MISO, LSM9DS1_MOSI, LSM9DS1_XGCS, LSM9DS1_MCS);
// Or hardware SPI! In this case, only CS pins are passed in
//Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

float magx0 = 0;
float magy0 = 0;
float magz0 = 0;
float magxM = 0;
float magyM = 0;
float magzM = 0;

const int buttonpin = 3;

float angX = 0.0;
float angY = 0.0;
float angZ = 0.0;

//float vx = 0.0;
//float vy = 0.0;
//float vz = 0.0;

float mpux = 0.0;
float mpuy = 0.0;
float mpuz = 0.0;

void setupSensor() {
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);

  // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}


void setup() {
  pinMode(buttonpin, INPUT);

  Serial.begin(115200);
  while (!Serial) {
    delay(10); // will pause Zero, Leonardo, etc until serial console opens
  }

  // Try to initialize MPU6050
  if (!mpu.begin()) {
//    Serial.println("Failed to find MPU6050 chip.");
    while (1) {
      delay(10);
    }
  }

  // Try to initialize LSM9DS1
  if (!lsm.begin()) {
//    Serial.println("Failed to initialize the LSM9DS1.");
    while (1);
  }
//  Serial.println("Found LSM9DS1 9DOF");

  // Set ranges for MPU6050
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
//  Serial.println("");
  delay(100);

  // Setup magnemtometer
//  Serial.println("Initializing... remove any nearby magnets from magnetometer sensor...");
  delay(3000);

  // helper to just set the default scaling we want, see above!
  setupSensor();

  lsm.read();  /* ask it to read in the data */

  /* Get a new sensor event */
  sensors_event_t a, m, g, t;
  lsm.getEvent(&a, &m, &g, &t);

  magx0 = m.magnetic.x;
  magy0 = m.magnetic.y;
  magz0 = m.magnetic.z;

//  Serial.println("Finished initializing.");
  delay(1000);
}

void loop() {
  lsm.read();  /* ask it to read in the data */

  /* Get a new sensor event */
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);
  mpu.getEvent(&a, &g, &temp);

  //MPU6050 Data
  float accx = a.acceleration.x;
  float accy = a.acceleration.y;
  float accz = a.acceleration.z;
  float roll = g.gyro.x;
  float pitch = g.gyro.y;
  float yaw = g.gyro.z;

  // Calculate angle relative to starting angle
  angX += roll * 0.03;
  angY += pitch * 0.03;
  angZ += yaw * 0.03;

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

  //  if (abs(accx) >= 0.75) {vx += accx * 0.03;}
  //  if (abs(accy) >= 0.75) {vy += accy * 0.03;}
  //  if (abs(accz) >= 0.75) {vz += accz * 0.03;}

  if (abs(accx) >= 1) {
    //    mpux += vx * 0.03 + 0.5 * accx * pow(0.03,2);
    mpux += accx * 0.03;
  }
  if (abs(accy) >= 1) {
    //    mpuy += vy * 0.03 + 0.5 * accy * pow(0.03,2);
    mpuy += accy * 0.03;
  }
  if (abs(accz) >= 1.25) {
    //    mpuz += vz * 0.03 + 0.5 * accz * pow(0.03,2);
    mpuz += accz * 0.03;
  }

  // Magnetometer Data
  float magx = m.magnetic.x - magx0;
  float magy = m.magnetic.y - magy0;
  float magz = m.magnetic.z - magz0;

  magx = pow(abs(magx), 0.5);
  magy = pow(abs(magy), 0.5);
  magz = pow(abs(magz), 0.5);

  // Scale down the magnetometer position values & reposition MPU if it strays
  magx = magx/8;
  magy = magy/8;
  magz = magz/8;

  if(mpux > 8) {mpux = 0;}
  if(mpuy > 8) {mpuy = 0;}
  if(mpuz > 8) {mpuz = 0;}

  // Weighted average of mag and mpu positions
  float magweightx = 0.0;
  float magweightz = 0.0;
  if (magx <= 6.5) {magweightx = magx/6.5;} else {magweightx = 1;}
  if (magz <= 6.5) {magweightz = magz/6.5;} else {magweightz = 1;}
  
  float x = magweightx*magx + (1-magweightx)*mpux;
  float y = magweightz*magz + (1-magweightz)*mpuy;

  // Print out the values
  if (digitalRead(buttonpin) == HIGH) {Serial.print("1,");} else {Serial.print("0,");}
  Serial.print(abs(x));        Serial.print(",");
  Serial.print(abs(y));        Serial.print(",");
  Serial.print(mpux);     Serial.print(",");
  Serial.print(mpuy);     Serial.print(",");
  Serial.print(mpuz);     Serial.print(",");
  Serial.print(magx);     Serial.print(",");
  Serial.print(magy);     Serial.print(",");
  Serial.print(magz);     Serial.print(",");
  Serial.print(angX);     Serial.print(",");
  Serial.print(angY);     Serial.print(",");
  Serial.print(angZ);     Serial.println();

  delay(30);
}
