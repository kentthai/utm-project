#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
//#include "vn/sensors.h"
#include <tf/transform_broadcaster.h>
#include <utm_converter/Utm.h>

#include <sstream>

void vnimuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  static tf::TransformBroadcaster broadcaster;
 
  broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w), tf::Vector3(0,0,0)), msg->header.stamp, "Sensor", "local"));
  
  ROS_INFO("x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w );
}

void utmOffsetCallback(const utm_converter::Utm::ConstPtr& msg)
{
  static tf::TransformBroadcaster broadcaster;
  
  broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(msg->northing, msg->easting, msg->altitude)), msg->header.stamp, "local", "global"));

//  broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(msg->northing-orig_lla.x*110902.,msg->easting-orig_lla.y*93578.,msg->altitude-orig_lla.z)), ros::Time::now(), "local", "global"));
  ROS_INFO("UTM Northing: [%f] Easting: [%f] Altitude: [%f]", msg->northing, msg->easting, msg->altitude);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tftester");

  ros::NodeHandle n;
  
  ros::Subscriber imu_sub = n.subscribe("vnimu", 1000, vnimuCallback);
  /*
  ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("vnimu", 1000, 
    boost::bind(vnimuCallback, _1, broadcaster));
  */

  ros::Subscriber utm_sub = n.subscribe("utmOffset", 1000, utmOffsetCallback);

  ros::spin();

  return 0;
}

