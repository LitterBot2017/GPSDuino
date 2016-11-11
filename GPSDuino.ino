#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <gps_msgs/GPS_msg.h>
#include <gps_msgs/IMU_msg.h>
#include <gps_msgs/Accel_msg.h>
#include <gps_msgs/Gyro_msg.h>
#include <gps_msgs/Mag_msg.h>
#include <TinyGPS.h>
#include "PID.h"
//#include <SoftwareSerial.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <RoboClaw.h>

RoboClaw motor_control(&Serial2,10000);


ros::NodeHandle arduinoNode;

#define LSM9DS1_M 0x1E; //Magnetometer address
#define LSM9DS1_AG 0x6B; //Accelerometer and gyro address
#define DECLINATION -9.19
#define mc_address 0x81


TinyGPS gps;  
LSM9DS1 imu;
long oldTime=millis();

gps_msgs::GPS_msg GPS_Data;
gps_msgs::IMU_msg IMU_Data;
//geometry_msgs::Twist vel_data;

ros::Publisher GPS_pub("gps_data",&GPS_Data);
ros::Publisher IMU_pub("imu_data",&IMU_Data);
//ros::Publisher vel_pub("cmd_vel",&vel_data);

void motor_com_cb(const geometry_msgs::Twist& in_msg)
{
  if(in_msg.linear.x>0)
  {
    motor_control.ForwardMixed(mc_address, (int) in_msg.linear.x);
  }
  if(in_msg.linear.x<0)
  {
    motor_control.BackwardMixed(mc_address, (int)-1*in_msg.linear.x);
  }
  if(in_msg.angular.z<0)
  {
    motor_control.TurnLeftMixed(mc_address, -1*(int)in_msg.angular.z*(int)in_msg.linear.z);
  }
  if(in_msg.angular.z>0)
  {
    motor_control.TurnRightMixed(mc_address, (int)in_msg.angular.z*(int)in_msg.linear.z);
  }
  if(in_msg.angular.z==0&&in_msg.linear.x==0)
  {
    motor_control.ForwardMixed(mc_address, 0);
    motor_control.TurnRightMixed(mc_address, 0);
  }
}

ros::Subscriber <geometry_msgs::Twist> input_from_pc("cmd_vel",&motor_com_cb);
/************** Arduino Specific *********************/
PID test=PID(0,10,0.01,0,0,10,-10);
void setup() {
  // ROS Initialization
  arduinoNode.initNode();
  arduinoNode.advertise(GPS_pub);
  arduinoNode.advertise(IMU_pub);  
//  arduinoNode.advertise(vel_pub);
  arduinoNode.subscribe(input_from_pc);
  Serial1.begin(9600);
  motor_control.begin(38400);
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress=LSM9DS1_M;
  imu.settings.device.agAddress=LSM9DS1_AG;
  if (!imu.begin())
  {
    IMU_Data.imu_ready=1;
    while(1)
    ;
  }
}



void readaccel()
{
  imu.readAccel();
  IMU_Data.accel.x=imu.calcAccel(imu.ax);
  IMU_Data.accel.y=imu.calcAccel(imu.ay);
  IMU_Data.accel.z=imu.calcAccel(imu.az);
}
void readgyro()
{
  imu.readGyro();
  IMU_Data.gyro.x=imu.calcGyro(imu.gx);
  IMU_Data.gyro.y=imu.calcGyro(imu.gy);
  IMU_Data.gyro.z=imu.calcGyro(imu.gz);
}

void readmag()
{
   imu.readMag();
  float heading;
  if (imu.my == 0)
    heading = (imu.mx < 0) ? 180.0 : 0;
  else
    heading = atan2(-imu.mx, -imu.my);
    
  heading -= (DECLINATION / 180)* PI ;
  
  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;
  heading *= 180.0 / PI;

  IMU_Data.mag.x=heading;
  //IMU_Data.mag.y=imu.calcMag(imu.my);
  IMU_Data.mag.z=imu.calcMag(imu.mz);
  float elapsedTime=(float)(millis()-oldTime);
  oldTime=millis();
  float newSpeed=test.getNewValue(heading,40,elapsedTime);
  IMU_Data.mag.y=newSpeed;
  if(newSpeed>=0)
  {    
    motor_control.TurnLeftMixed(mc_address, newSpeed);
  }
  else
  {
    motor_control.TurnLeftMixed(mc_address, -1*newSpeed);
  }  
}

void loop() {
  /* Comment the following section to activate keyboard teleop */
  while(Serial1.available())
  {
    int c=Serial1.read();
    if(gps.encode(c))
    {
      float lat_l;
      float long_l;
      unsigned long fix_time_l;
      gps.f_get_position(&lat_l,&long_l,&fix_time_l);
      unsigned short sats=gps.satellites();
      float speed_val=gps.f_speed_mph();
      GPS_Data.lattitude=lat_l;
      GPS_Data.longitude=long_l;
      GPS_Data.fix_time=fix_time_l;
      GPS_Data.sats=sats;
      
    }
    GPS_Data.speed_val=c;
  }
  readaccel();
  readgyro();
  readmag();
  IMU_pub.publish(&IMU_Data);
  /* Connent end point for keyboard teleop*/
  GPS_pub.publish(&GPS_Data);
  //vel_pub.publish(&vel_data);
  arduinoNode.spinOnce();
  delay(100);
}
