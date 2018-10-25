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



#ifndef _IMU_H_
#define _IMU_H_

#include "Arduino.h"
#include<Wire.h>

#define	MPU_ADDR	0x68              
#define	RAD_TO_DEG	57.2728
#define	DEG_TO_RAD	0.01746

#define	ACCELERATION_DUE_GRAVITY	9.8
#define	POWER_MNGMT_ADDR	0x6B

#define	ACCL_STARTING_ADDR	0x3B
#define	GYRO_STARTING_ADDR	0x43
#define	TEMP_STARTING_ADDR	0x41

#define	GYRO_CONF_ADDR	0x1B
#define	ACCL_CONF_ADDR	0x1C


class IMU
{
   private:

   float yaw , pitch , roll , ACCEL_CONVERTER , GYRO_CONVERTER ;
   bool set_gyro_angle ;
   
   struct actual_value
   {
      int16_t x ;
      int16_t y ;
      int16_t z ;
   }accel_actual , gyro_actual;

   struct processed_value
   {
      float x ;
      float y ;
      float z ;
   }accel_processed , gyro_processed , angle_accel , angle_gyro , mean_gyro;

   struct Thermometer
   {
      int16_t temperature ;
   }thermometer;
   
   public :
   
   IMU();
   
   void begin();
   void begin(uint8_t , uint8_t);
   
   void ReadAccelerometer();
   void ReadGyroscope();
   void ReadTemperature();
   
   void processGyroData();
   void processAccelData();
   
   void callibrateSensors();
   
   void calculateGyroAngle(float);
   void calculateAccelAngle(float , float);
   void calculateCompAngles(float);

   float getAccel_x();
   float getAccel_y();
   float getAccel_z();

   float getGyro_x();
   float getGyro_y();
   float getGyro_z();

   float getAccelAngle_x();
   float getAccelAngle_y();
   float getAccelAngle_z();

   float getGyroAngle_x();
   float getGyroAngle_y();
   float getGyroAngle_z();

   float getCompAngle_yaw();
   float getCompAngle_pitch();
   float getCompAngle_roll();

};

#endif

