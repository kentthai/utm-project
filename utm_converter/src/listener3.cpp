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

ros::NodeHandle *nptr;

void chatterCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg)
{

  // Converts LL to UTM
  RobotLocalization::NavsatConversions::LLtoUTM( gps_msg->latitude, gps_msg->longitude, northing, easting, myStr);
  // Copy over altitude from msg
  altitude = gps_msg->altitude;

  // Create publisher and populate msg
  static ros::Publisher utm_pub = nptr->advertise<utm_converter::Utm>("utm", 1000);

  utm_converter::Utm utm_msg;

  utm_msg.header.stamp = gps_msg->header.stamp;
  utm_msg.header.frame_id = "UTM";
  utm_msg.utm_zone = myStr;
  utm_msg.northing = northing;
  utm_msg.easting = easting;
  utm_msg.altitude = altitude;
    
  utm_pub.publish(utm_msg);

  ROS_INFO("UTM Zone: [%s], Northing: [%f], Easting: [%f], Altitude: [%f]", myStr.c_str(), northing, easting, altitude);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  nptr = &n;

  //Parameterize topic name
  std::string topicname;
  ros::param::get("~topic", topicname);

  // Printing to test if topicname was successfully parameterized
  //ROS_INFO("topicname: %s", topicname.c_str());

  ros::Subscriber sub = n.subscribe( topicname, 1000, chatterCallback);

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    ++count;
  }

  ros::spin();

  return 0;
}
