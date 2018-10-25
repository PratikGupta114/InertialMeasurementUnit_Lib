/*
 * MPU-6050 Value Reading code for Arduino.
 * 25/10/2018 by Pratik Gupta <rgrgupta1148@gmail.com>
 * MPU-6050 demonstration Arduino Sketch
 * 
 * Following are the explanations of the various functions which can be availed 
 * using this library
 * 
 * void begin();
   void begin(uint16_t , uint16_t);            // For initializing the IMU 
   
   void ReadAccelerometer();                   
   void ReadGyroscope();
   void ReadTemperature();
   
   void processGyroData();                     // for converting raw Gyro values to degrees per second
   void processAccelData();                    // for converting raw accelerometer values to 'g' units.
   
   void callibrateSensors();                   // for calculating and subtracting the gyroscope offset.
   
   void calculateGyroAngle(float);             // the parameter sets the dt (dime difference for one cycle of the code)
   void calculateAccelAngle(float , float);    // the two parameters are correction of the x and y axis angles respectively
   void calculateCompAngles(float);            // the parameter sets the weightage for the gyroscope and its complimentary weight for the Accelerometer (Complimentary filter)

   float getAccel_x();                         // Reading Accelerometer values (Processed)
   float getAccel_y();
   float getAccel_z();

   float getGyro_x();                          // Reading Gyroscope values (Processed)
   float getGyro_y();
   float getGyro_z();

   float getAccelAngle_x();                    // Reading Accelerometer Angle
   float getAccelAngle_y();
   float getAccelAngle_z();

   float getGyroAngle_x();                     // Reading Gyoscope Angle
   float getGyroAngle_y();
   float getGyroAngle_z();

   float getCompAngle_yaw();                   // Reading Complimentary Angles
   float getCompAngle_pitch();
   float getCompAngle_roll();
 */

/*
 * Following are the list of possible accelerometer and gyroscope configurations
 * 
 * Configuration_value    Accelerometer_range   Gyroscope_range
 *        0x00                    ±2g             ±250deg/sec
 *        0x08                    ±4g             ±500deg/sec     
 *        0x10                    ±8g             ±1000deg/sec
 *        0x18                    ±16g            ±2000deg/sec
 *        
 * NOTE : by increasing the range of detection of accelerometer and gyroscope values
 *        the sensitivity of the device decreases as well.
 */

//#define ACCELEROMETER_CONFIGURATION  0x00
//#define GYROSCOPE_CONFIGURATION      0x00

#include<IMU.h>
#include<Wire.h>

IMU imu;

struct angle_data
{
  float x = 0.0;
  float y = 0.0;
  float z = 0.0;
}accel_angle , gyro_angle;                    // Data Structure makes it more convenient

float difference = 0.0;                       // Stores the time taken in one cycle.
int i = 0;
unsigned long long previousTime = 0;

void setup() 
{
  previousTime = millis();
  Serial.begin(9600);
  imu.begin();                                // Initializes the MPU-6050
  //imu.begin(ACCELEROMETER_CONFIGURATION , GYROSCOPE_CONFIGURATION ); // uncomment this line and comment the above line to adjust te range of accelerometer and gyroscope

  imu.callibrateSensors();                    // Callibrates the gyroscope
                                              // MPU-6050 should be kept horizontal while callibration.
}

void loop() 
{
     imu.ReadAccelerometer();                 // <- This function and
     imu.processAccelData();                  // <- this function should be called before fetching the values of the accelerometer
     imu.calculateAccelAngle(0.0,0.0);        // the parameters are the values for angle correction for x and y axis respectively
     accel_angle.x = imu.getAccelAngle_x();
     accel_angle.y = imu.getAccelAngle_y();
     accel_angle.z = imu.getAccelAngle_z();

     imu.ReadGyroscope();                     // <- This function and
     imu.processGyroData();                   // <- this function shall also be called before fetching the Gyroscope values
     difference = millis() - previousTime;
     previousTime = millis();
     imu.calculateGyroAngle(difference * 1.0);// parameter sets the dt
     gyro_angle.x = imu.getGyroAngle_x();
     gyro_angle.y = imu.getGyroAngle_y();
     gyro_angle.z = imu.getGyroAngle_z();

     Serial.print(accel_angle.x);             
     Serial.print("\t");
     Serial.print(accel_angle.y);
     Serial.print("\t");
     Serial.print(accel_angle.z);
     Serial.print("\t");
     Serial.print(gyro_angle.x);
     Serial.print("\t");
     Serial.print(gyro_angle.y);
     Serial.print("\t");
     Serial.println(gyro_angle.z);
}
