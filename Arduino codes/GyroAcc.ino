#include <Wire.h>
#include "Kalman.h" 
Kalman kalmanX;
Kalman kalmanY;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Gyroscope angle
double kalAngleX, kalAngleY; // Angle after Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

//////////////////////////////////////////////////////////////////////////////////////

void GyroAccSetup()
{
  Wire.begin();  // address is not specified, so arduino joins the bus as a master.
  
  while (i2cWrite(0x6B, 0x01, false));  //PWR_MGMT_1 register, PLL with X axis gyroscope reference and disable sleep mode
                                        //Should be able to disable temp measurement.
  
  //Set data to be written on 0x19 register.
  i2cData[0] = 7; // Set the sample rate to 1000Hz: 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; //Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; //Set Gyro Full Scale Range to 250deg/s
  i2cData[3] = 0x00; //Set Accelerometer Full Scale Range to 2g

  while (i2cWrite(0x19, i2cData, 4, true));

  //Verify we I2C is working with IMU.
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68)  //Read "WHO_AM_I" register
  { 
    Serial.print(F("Error reading sensor"));
    while (1);  //This line prevents the following codes from running, because an error has been detected above.
  }
  delay(100); //Wait for sensor to stabilize

  //Read the inertial force vector from acc.
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];
  
  double roll  = atan2(accY, accZ)* RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  
  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  
  gyroXangle = roll;
  gyroYangle = pitch;
  
  timer = micros();
}

////////////////////////////////////////////////////////////////////////////////////

double GyroAccCompute()
{
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];
  // Calculate delta time
  double dt = (double)(micros() - timer) / 1000000;   //The timer is always the time it is last called since start up.
  timer = micros();  //The timer gets updated in every iteration of the loop.

  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

 if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;

  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  delay(2);

  /*
  Serial.print("roll: "); Serial.print(roll);
  Serial.print(" | : kalAngleX"); Serial.print(kalAngleX);
  Serial.print(" | : gyroXangle"); Serial.println(gyroXangle);
  */
  Serial.println(kalAngleX);
  return kalAngleX;
}

