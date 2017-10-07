#include <SPI.h>
#include <Wire.h>
#include <SFE_LSM9DS0.h>

#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);

const byte INT1XM = 2; // INT1XM tells us when accel data is ready
const byte INT2XM = 3; // INT2XM tells us when mag data is ready
const byte DRDYG = 4;  // DRDYG tells us when gyro data is ready

const float BIAS_GYRO_X = -0.5439;
const float BIAS_GYRO_Y = 7.1401;
const float BIAS_GYRO_Z = -10.3321;

void setup() {
  // Set up interrupt pins as inputs:
  pinMode(INT1XM, INPUT);
  pinMode(INT2XM, INPUT);
  pinMode(DRDYG, INPUT);

  Serial.begin(115200);

  Serial.println("time, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z");

  dof.begin();
  dof.setAccelScale(dof.A_SCALE_2G);
  dof.setGyroScale(dof.G_SCALE_2000DPS);
}

void loop() {
  if(digitalRead(DRDYG) && digitalRead(INT1XM)) {
    auto time = millis();
    
    dof.readGyro();
    dof.readAccel();

    Serial.print(time);
    Serial.print(", ");  
    Serial.print(dof.calcGyro(dof.gx) - BIAS_GYRO_X);
    Serial.print(", ");
    Serial.print(dof.calcGyro(dof.gy) - BIAS_GYRO_Y);
    Serial.print(", ");
    Serial.print(dof.calcGyro(dof.gz) - BIAS_GYRO_Z);
    Serial.print(", ");
    Serial.print(dof.calcAccel(dof.ax));
    Serial.print(", ");
    Serial.print(dof.calcAccel(dof.ay));
    Serial.print(", ");
    Serial.println(dof.calcAccel(dof.az));
  }

}
