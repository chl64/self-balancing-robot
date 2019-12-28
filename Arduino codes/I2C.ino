#include <Wire.h>

const uint8_t IMUAddress=0x68;  //I2C address of the MPU-6050
const uint16_t I2C_TIMEOUT = 1000; //Used to check for errors in I2C communication

uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) 
{
  return i2cWrite(registerAddress, &data, 1, sendStop); //Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) 
{
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); //Returns 0 on success
  if (rcode) 
  {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode;
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) 
{
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) 
  {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode;
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true);  //Wire.requestFrom(address, no of bytes to request, stop)
                                                        // stop = true will send a stop message after the request, releasing the bus.
  for (uint8_t i = 0; i < nbytes; i++) 
  {
    // if there are still bytes to be read.
    if (Wire.available())  // code will be executed if Wire.available() != 0.
    {  
      data[i] = Wire.read();
    }
    // in case if "i" has not reached nbytes, but Wire.available() gives zero 
    // because of some timeout problem.
    // So try to wait for the timeout to pass.
    else 
    {
      timeOutTimer = micros();
      
      // while the time passed by from the previous code is less than 1000ms,
      // AND while there is no available bytes to be read.
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());  //I2C_TIMEOUT == 1000ms

      // if this temporary problem has disappeared while waititng.
      if (Wire.available())
      {  
        data[i] = Wire.read();
      }
      else 
      {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}

