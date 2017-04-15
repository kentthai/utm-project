#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "robot_localization/navsat_conversions.h"
#include <utm_converter/Utm.h>


#include <string>
using namespace std;

string myStr;
double x;
double &northing=x;
double y;
double &easting=y;
double altitude;

void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{


  RobotLocalization::NavsatConversions::LLtoUTM( gps_msg->latitude, gps_msg->longitude, northing, easting, myStr);

  altitude = gps_msg->altitude;

  ROS_INFO("UTM Zone: [%s], Northing: [%f], Easting: [%f], Altitude: [%f]", myStr.c_str(), northing, easting, altitude);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe( "vngps" , 1000, chatterCallback);

  ros::Publisher utm_pub = n.advertise<utm_converter::Utm>("utm", 1000);
 
  ros::Rate loop_rate(100);

  int count = 0;
  while (ros::ok())
  {
    utm_converter::Utm utm_msg;

    utm_msg.header.stamp = ros::Time::now();
    utm_msg.header.frame_id = "UTM";
    utm_msg.utm_zone = myStr;
    utm_msg.northing = northing;
    utm_msg.easting = easting;
    utm_msg.altitude = altitude;
    
    utm_pub.publish(utm_msg);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  ros::spin();

  return 0;
}
