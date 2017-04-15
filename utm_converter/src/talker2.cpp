#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher gps_pub = n.advertise<sensor_msgs::NavSatFix>("vngps", 1000);
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("vnimu", 1000);  
  ros::Publisher control_pub = n.advertise<std_msgs::Bool>("control", 1000);

  ros::Rate loop_rate(10);
  
  int controlCountdown = 10;
  int count = 0;
  while (ros::ok())
  {
    // gps msg
    sensor_msgs::NavSatFix gps_msg;

    //ros::Time time;
    gps_msg.header.stamp = ros::Time::now();
    gps_msg.header.frame_id = "Sensor";
    gps_msg.latitude = count%90;
    gps_msg.longitude = count%180;
    gps_msg.altitude = (count%100)+40;
    gps_msg.status.status = 0;

    ROS_INFO("Latitude: %f, Longitude: %f, Altitude: %f", gps_msg.latitude, gps_msg.longitude, gps_msg.altitude);

    // vnimu msg
    sensor_msgs::Imu msg;
    
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "Sensor";
    msg.orientation.x = count%10;
    msg.orientation.y = count%10;
    msg.orientation.z = count%10;
    msg.orientation.w = count%10;
    
    ROS_INFO("x: %f, y: %f, z: %f, w: %f", msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    
    // control msg
    std_msgs::Bool control_msg;
    
    control_msg.data = (controlCountdown == 0);
    if(controlCountdown == 0){
        controlCountdown = 10;
    }
    
    if(control_msg.data){
        ROS_INFO("Control Msg: TRUE");
    }
    else{
        ROS_INFO("Control Msg: FALSE");
    }

    // publish msgs
    gps_pub.publish(gps_msg);
    imu_pub.publish(msg);
    control_pub.publish(control_msg);

    ros::spinOnce();

    loop_rate.sleep();

    ++count;
    controlCountdown--;
  }

  return 0;
}
