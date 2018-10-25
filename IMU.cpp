/*********************************************************************************
 * IMU library collection - MPU - 6050 I2C device class
 * Based on InvenSense MPU-6050 regiter map document rev. 2.0 , 5/19/2011 (RM-MPU-6000A-00)
 * 25/10/2018 by Pratik Gupta <rgrgupta1148@gmail.com>
 * 
 * NOTE : THIS IS ONLY A PARTIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
 * DEVLOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF 
 * YOU DECIDE TO USE THS PARTICULAR CODE FOR ANYTHING.
 * 
 * ===============================================================================
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:
   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.
   
   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
   THE SOFTWARE.

   ==============================================================================
 */




#include "Arduino.h"
#include "IMU.h"

IMU :: IMU()
{
  yaw = 0.0;
  pitch = 0.0;
  roll = 0.0;
  ACCEL_CONVERTER = 1.0;
  GYRO_CONVERTER = 1.0;
  
  accel_actual.x = 0;
  accel_actual.y = 0;
  accel_actual.z = 0;
  
  gyro_actual.x = 0;
  gyro_actual.y = 0;
  gyro_actual.z = 0;
  
  accel_processed.x = 0.0f;
  accel_processed.y = 0.0f;
  accel_processed.z = 0.0f;
  
  gyro_processed.x = 0.0f;
  gyro_processed.y = 0.0f;
  gyro_processed.z = 0.0f;
  
  angle_accel.x = 0.0f;
  angle_accel.y = 0.0f;
  angle_accel.z = 0.0f;
  
  angle_gyro.x = 0.0f;
  angle_gyro.y = 0.0f;
  angle_gyro.z = 0.0f;
  
  mean_gyro.x = 0.0f;
  mean_gyro.y = 0.0f;
  mean_gyro.z = 0.0f;
  
  thermometer.temperature = 0;
  set_gyro_angle = false;
}


void IMU :: begin()
{
  Wire.begin();
  Wire.setClock(400000UL);

  ACCEL_CONVERTER = 16384;
  GYRO_CONVERTER = 131;
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(POWER_MNGMT_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCL_CONF_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONF_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);
}

void IMU :: begin(uint8_t accel_conf, uint8_t gyro_conf)
{
  Wire.begin();
  Wire.setClock(400000UL);

  switch(accel_conf)
  {
    case 0x00:
    {
       ACCEL_CONVERTER = 16384 ;
    }
    case 0x80:
    {
       ACCEL_CONVERTER = 8192;
    }
    case 0x10:
    {
       ACCEL_CONVERTER = 4096;
    }
    case 0x18:
    {
       ACCEL_CONVERTER = 2048;
    }
  }

  switch(gyro_conf)
  {
    case 0x00:
    {
       GYRO_CONVERTER = 131;
    }
    case 0x80:
    {
       GYRO_CONVERTER = 65.5;
    }
    case 0x10:
    {
       GYRO_CONVERTER = 32.8;
    }
    case 0x18:
    {
       GYRO_CONVERTER = 16.4;
    }
  }
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(POWER_MNGMT_ADDR);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCL_CONF_ADDR);
  Wire.write(accel_conf);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONF_ADDR);
  Wire.write(gyro_conf);
  Wire.endTransmission();
  delay(10);
}

void IMU :: ReadAccelerometer()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(ACCL_STARTING_ADDR);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDR , 6);

    while (Wire.available() < 6);

    accel_actual.x = Wire.read() << 8 | Wire.read();
    accel_actual.y = Wire.read() << 8 | Wire.read();
    accel_actual.z = Wire.read() << 8 | Wire.read();
}

void IMU :: ReadGyroscope()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_STARTING_ADDR);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDR , 6);

    while (Wire.available() < 6);

    gyro_actual.x = Wire.read() << 8 | Wire.read();
    gyro_actual.y = Wire.read() << 8 | Wire.read();
    gyro_actual.z = Wire.read() << 8 | Wire.read();
}

void IMU :: ReadTemperature()
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(TEMP_STARTING_ADDR);
    Wire.endTransmission();
    Wire.requestFrom(MPU_ADDR , 2);

    while (Wire.available() < 2);

    thermometer.temperature = Wire.read() << 8 | Wire.read();
}

void IMU :: processGyroData()
{
    gyro_processed.x = (gyro_actual.x / GYRO_CONVERTER);
    gyro_processed.y = (gyro_actual.y / GYRO_CONVERTER);
    gyro_processed.z = (gyro_actual.z / GYRO_CONVERTER);
}

void IMU :: processAccelData()
{
    accel_processed.x = ((accel_actual.x * ACCELERATION_DUE_GRAVITY) / ACCEL_CONVERTER) ;
    accel_processed.y = ((accel_actual.y * ACCELERATION_DUE_GRAVITY) / ACCEL_CONVERTER) ;
    accel_processed.z = ((accel_actual.z * ACCELERATION_DUE_GRAVITY) / ACCEL_CONVERTER) ;
}

void IMU :: callibrateSensors()
{
    int i = 0;
    for(i = 1 ; i <= 800 ; i++)
    {
      ReadGyroscope();
      processGyroData();
      
      mean_gyro.x += gyro_processed.x;
      mean_gyro.y += gyro_processed.y;
      mean_gyro.z += gyro_processed.z;
    }
    mean_gyro.x /= (float)i;
    mean_gyro.y /= (float)i;
    mean_gyro.z /= (float)i;
}

void IMU :: calculateGyroAngle(float dt)
{
   angle_gyro.x += ( (gyro_processed.x - mean_gyro.x) / (1000 / dt)) ;
   angle_gyro.y += ( (gyro_processed.y - mean_gyro.y) / (1000 / dt)) ;
   angle_gyro.z += ( (gyro_processed.z - mean_gyro.z) / (1000 / dt)) ;
}

void IMU :: calculateAccelAngle(float error_x , float error_y)
{
   angle_accel.x = (atan( (accel_processed.y) / sqrt( (accel_processed.x * accel_processed.x) + (accel_processed.z * accel_processed.z) ) ) * RAD_TO_DEG ) - error_x;
   angle_accel.y = (atan( (accel_processed.x) / sqrt( (accel_processed.y * accel_processed.y) + (accel_processed.z * accel_processed.z) ) ) * RAD_TO_DEG ) - error_y;
}

void IMU :: calculateCompAngles(float weight)
{
   if(set_gyro_angle == true)
   {
      pitch = (angle_gyro.y * weight) + (angle_accel.y * (1.0 - weight));
      roll  = (angle_gyro.x * weight) + (angle_accel.x * (1.0 - weight));
      yaw   = (angle_gyro.z); 
   }
   else
   {
      pitch = angle_accel.y;
      roll  = angle_accel.x;
      yaw   = angle_accel.z;
      set_gyro_angle = true;
   }
}

float IMU :: getAccel_x()
{
  return accel_processed.x;
}

float IMU :: getAccel_y()
{
  return accel_processed.y;
}

float IMU :: getAccel_z()
{
  return accel_processed.z;
}

float IMU :: getGyro_x()
{
  gyro_processed.x -= mean_gyro.x;
  return gyro_processed.x;
}

float IMU :: getGyro_y()
{
  gyro_processed.y -= mean_gyro.y;
  return gyro_processed.y;
}

float IMU :: getGyro_z()
{
  gyro_processed.z -= mean_gyro.z;
  return gyro_processed.z;
}

float IMU :: getAccelAngle_x()
{
    return angle_accel.x ;
}

float IMU :: getAccelAngle_y()
{
    return angle_accel.y ;
}

float IMU :: getAccelAngle_z()
{
    return angle_accel.z;
}

float IMU :: getGyroAngle_x()
{
    return angle_gyro.x ;
}

float IMU :: getGyroAngle_y()
{
    return angle_gyro.y ;
}

float IMU :: getGyroAngle_z()
{
    return angle_gyro.z  ;
}

float IMU :: getCompAngle_yaw()
{
    return yaw ;
}

float IMU :: getCompAngle_pitch()
{
    return pitch;
}
float IMU :: getCompAngle_roll()
{
    return roll ;
}

