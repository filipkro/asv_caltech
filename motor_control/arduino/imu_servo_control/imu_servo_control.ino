#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Mahony.h>
#include <Madgwick.h>
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
//#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32.h>
#include <Servo.h> 
#include <motor_control/Arduino_imu.h>

// Note: This sketch is a WORK IN PROGRESS

#define ST_LSM303DLHC_L3GD20        (0)
#define ST_LSM9DS1                  (1)
#define NXP_FXOS8700_FXAS21002      (2)

// Define your target sensor(s) here based on the list above!
// #define AHRS_VARIANT    ST_LSM303DLHC_L3GD20
#define AHRS_VARIANT   NXP_FXOS8700_FXAS21002

#if AHRS_VARIANT == ST_LSM303DLHC_L3GD20
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#elif AHRS_VARIANT == ST_LSM9DS1
// ToDo!
#elif AHRS_VARIANT == NXP_FXOS8700_FXAS21002
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#else
#error "AHRS_VARIANT undefined! Please select a target sensor combination!"
#endif


// Create sensor instances.
#if AHRS_VARIANT == ST_LSM303DLHC_L3GD20
Adafruit_L3GD20_Unified       gyro(20);
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
#elif AHRS_VARIANT == ST_LSM9DS1
// ToDo!
#elif AHRS_VARIANT == NXP_FXOS8700_FXAS21002
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
#endif

// Mag calibration values are calculated via ahrs_calibration.
// These values must be determined for each board/environment.
// See the image in this sketch folder for the values used
// below.

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { -79.8F, -91.36F, 78.56F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.996,  -0.036,  0.004 },
                                    {  -0.036,  0.983, 0.008 },
                                    {  0.004, 0.008,  1.023 } };

float mag_field_strength        = 39.73F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3]      = { 0.0F, 0.0F, 0.0F };

// Mahony is lighter weight as a filter and should be used
// on slower systems
Mahony filter;
//Madgwick filter;


// ROS and Servo setup 

#define SERVOPIN 11
#define SERVO_MIN 1195
#define SERVO_MAX 1834 // turning right

ros::NodeHandle  nh;
//std_msgs::Float32MultiArray imu_msg;
motor_control::Arduino_imu imu_msg;
std_msgs::UInt32 servo_cmd;
ros::Publisher imu_pub("imu", &imu_msg);

Servo myservo;
int servoVal = 1600; // default is straight
int counter = 0; // count the number of time since last update

void servo_callback(const std_msgs::UInt32& msg) {
//  imu_pub.publish(&imu_msg);
  if (msg.data >= SERVO_MIN && msg.data <= SERVO_MAX) {
    servoVal = msg.data;
    myservo.writeMicroseconds(msg.data);
  } else {
    servoVal = 1600;
    myservo.writeMicroseconds(1600);
  }
  counter = 0;
}

ros::Subscriber<std_msgs::UInt32> servo_sub("servo_cmd", &servo_callback);
float imu_data[9]={0,0,0,0,0,0,0,0,0};

void setup()
{
  nh.initNode();
  imu_msg.data_length = 9;
  
  // publisher for imu
  nh.advertise(imu_pub);
  
  // subscriber for servo
  myservo.attach(SERVOPIN);
  nh.subscribe(servo_sub);  

  // Filter expects 70 samples per second
  // Based on a Bluefruit M0 Feather ... rate should be adjuted for other MCUs
  gyro.begin();
  accelmag.begin();
  filter.begin(10);

}

void loop(void)
{
  // write servo value, if there is not update for 
  // 500 milliseconds, return servoVal to default
  // position
  myservo.writeMicroseconds(servoVal);
  if(counter>50){ 
    servoVal = 1600;
  }
  counter++;
  
  sensor_t sensor;
  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  
  //gyro.getSensor(&sensor);
  // Get new data samples
  gyro.getEvent(&gyro_event);
  
#if AHRS_VARIANT == NXP_FXOS8700_FXAS21002
  accelmag.getEvent(&accel_event, &mag_event);
#else
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
#endif
  
  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x + gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y + gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z + gyro_zero_offsets[2];

  // The filter library expects gyro data in degrees/s, but adafruit sensor
  // uses rad/s so we need to convert them first (or adapt the filter lib
  // where they are being converted)
  gx *= 57.2958F;
  gy *= 57.2958F;
  gz *= 57.2958F;

  // Update the filter
  filter.update(gx*0.2, gy*0.2, gz*0.2,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);

  // Print the orientation filter output
  // Note: To avoid gimbal lock you should read quaternions not Euler
  // angles, but Euler angles are used here since they are easier to
  // understand looking at the raw values. See the ble fusion sketch for
  // and example of working with quaternion data.
  float roll = filter.getRoll() / 360.0 * M_PI;
  float pitch = filter.getPitch() / 360.0 * M_PI;
  float heading = filter.getYaw() / 360.0 * M_PI;
  
  imu_data[0] = accel_event.acceleration.x;
  imu_data[1] = accel_event.acceleration.y;
  imu_data[2] = accel_event.acceleration.z;
  imu_data[3] = gx;
  imu_data[4] = gy;
  imu_data[5] = gz;
  imu_data[6] = filter.getRollRadians();
  imu_data[7] = filter.getPitchRadians();
  imu_data[8] = filter.getYawRadians();
  
  imu_msg.data = imu_data;
  imu_msg.header.stamp = nh.now();
  imu_pub.publish(&imu_msg);
  nh.spinOnce();
  delay(10);

}
