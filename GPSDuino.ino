#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <gps_msgs/GPS_msg.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>

ros::NodeHandle arduinoNode;

TinyGPS gps;

gps_msgs::GPS_msg GPS_Data;

ros::Publisher GPS_pub("gps_data",&GPS_Data);
/************** Arduino Specific *********************/

void setup() {
  // ROS Initialization
  arduinoNode.initNode();
  arduinoNode.advertise(GPS_pub);
  Serial1.begin(9600);
}

void loop() {
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
      GPS_Data.speed_val=speed_val;
    }
  }
  GPS_pub.publish(&GPS_Data);
  arduinoNode.spinOnce();
  delay(100);
}
