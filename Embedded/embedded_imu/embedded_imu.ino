#include <SPI.h>
#include <Wire.h>
#include <SFE_LSM9DS0.h>

#include <math.h>

#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

const byte INT1XM = 2; // INT1XM tells us when accel data is ready
const byte INT2XM = 3; // INT2XM tells us when mag data is ready
const byte DRDYG = 4;  // DRDYG tells us when gyro data is ready

const float BIAS_GYRO_X = -0.5439;
const float BIAS_GYRO_Y = 7.1401;
const float BIAS_GYRO_Z = -10.3321;

const float ALPHA = 0.98f;

const byte START_BYTE = 0x00;

struct Angle {
  Angle()
    : pitch{0.f}
    , yaw{0.f}
    , roll{0.f} {
    
  }

  Angle(float _pitch, float _yaw, float _roll)
    : pitch{_pitch}
    , yaw{_yaw}
    , roll{_roll} {
    
  }
  
  float pitch, yaw, roll;
};

const byte MESSAGE_SIZE = (1 + sizeof(Angle));

float time;
Angle filtered;

void setup() {
  // Set up interrupt pins as inputs:
  pinMode(INT1XM, INPUT);
  pinMode(INT2XM, INPUT);
  pinMode(DRDYG, INPUT);

  Serial.begin(57600);

  dof.begin();
  dof.setAccelScale(dof.A_SCALE_2G);
  dof.setGyroScale(dof.G_SCALE_2000DPS);

  time = millis();
}

void loop() {
  if(digitalRead(DRDYG) && digitalRead(INT1XM)) {
    auto newTime = millis();
    auto dt = (newTime - time) / 1000.f;
    
    dof.readGyro();
    dof.readAccel();

    auto accel = getAccelAngle(dof);
    auto gyro = getGyroRate(dof);

    compFilter(ALPHA, filtered, getAccelAngle(dof), getGyroRate(dof), dt);
    
    sendAngle(filtered);

    time = newTime;
  }
}

Angle getAccelAngle(const LSM9DS0& imu) {
  float x = dof.calcAccel(dof.ax),
    y = dof.calcAccel(dof.ay),
    z = dof.calcAccel(dof.az);
  
  float pitch = 180.f / PI * atan2(x, mag(y, z)),
    roll = -180.f / PI * atan2(y, mag(x, z));

  return {pitch, 0.f, roll};
}

Angle getGyroRate(const LSM9DS0& imu) {
  return {-(dof.calcGyro(dof.gy) - BIAS_GYRO_Y), dof.calcGyro(dof.gz) - BIAS_GYRO_Z, -(dof.calcGyro(dof.gx) - BIAS_GYRO_X)};
}

void compFilter(const float& alpha, Angle& filtered, const Angle& accelAngle, const Angle& gyroRate, const float& dt) {
  filtered.pitch = alpha*(filtered.pitch + gyroRate.pitch*dt) + (1-alpha)*accelAngle.pitch;
  filtered.yaw += gyroRate.yaw*dt;
  filtered.roll = alpha*(filtered.roll + gyroRate.roll*dt) + (1-alpha)*accelAngle.roll;
}

float mag(const float& x, const float& y) {
  return sqrt(x*x + y*y);
}

void sendAngle(const Angle& angle) {
  byte message[MESSAGE_SIZE];

  message[0] = START_BYTE;
  memcpy(message+1, &angle, sizeof(angle));

  Serial.write(message, MESSAGE_SIZE);
  
}


