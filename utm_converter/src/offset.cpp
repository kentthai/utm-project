#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/NavSatFix.h"
#include "robot_localization/navsat_conversions.h"
#include <utm_converter/Utm.h>


#include <string>
using namespace std;

string utmZone;
double x;
double &northing=x;
double y;
double &easting=y;
double altitude;

double orig_northing = 0;
double orig_easting = 0;
double orig_altitude = 0;

bool controlReset = false;


ros::NodeHandle *nptr;

void resetOrigin(){
    orig_northing = northing;
    orig_easting = easting;
    orig_altitude = altitude;
}

void controlCallback(const std_msgs::Bool::ConstPtr& control_msg)
{
  controlReset = control_msg->data;

  if(controlReset){
    ROS_INFO("Reset: TRUE");
    resetOrigin();
  }
  else{
    ROS_INFO("Reset: FALSE");
  }
}

void utmCallback(const utm_converter::Utm::ConstPtr& utm_msg)
{
  static ros::Publisher utm_offset_pub = nptr->advertise<utm_converter::Utm>("utmOffset", 1000);

  utmZone = utm_msg->utm_zone;
  northing = utm_msg->northing;
  easting = utm_msg->easting;
  altitude = utm_msg->altitude;

  utm_converter::Utm utm_offset_msg;

  utm_offset_msg.header.stamp = utm_msg->header.stamp;
  utm_offset_msg.header.frame_id = "utmOffset";
  utm_offset_msg.utm_zone = utmZone;
  utm_offset_msg.northing = northing-orig_northing;
  utm_offset_msg.easting = easting-orig_easting;
  utm_offset_msg.altitude = altitude-orig_altitude;
  
  ROS_INFO("UTM Zone: [%s], Northing: [%f], Easting: [%f], Altitude: [%f]", utmZone.c_str(), utm_offset_msg.northing, utm_offset_msg.easting, utm_offset_msg.altitude);
    
  utm_offset_pub.publish(utm_offset_msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offset");

  ros::NodeHandle n;
  nptr = &n;

  ros::Subscriber control_sub = n.subscribe( "control" , 1000, controlCallback);
  ros::Subscriber utm_sub = n.subscribe( "utm" , 1000, utmCallback);

 
  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    ++count;
  }

  ros::spin();

  return 0;
}
