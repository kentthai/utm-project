#!/usr/bin/env/python
import roslib
roslib.load_manifest('vnimu_tf')
import rospy
import tf

#need to import msgs

#def handle_turtle_pose():
#    br = tf.TransformBroadcaster()
    #br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(att[0], att[1], att[2], att[3]), tf::Vector3(0,0,0)), ros::Time::now(), "Sensor", "local"));
    #br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(pos.x*110902.-orig_lla.x*110902.,pos.y*93578.-orig_lla.y*93578.,pos.z-orig_lla.z)), ros::Time::now(), "local", "global"));



if __name__ == '__main__':
    rospy.init_node('vn_tf_broadcaster')
    #rospy.Subscriber('/utm',
    #                 utm_converter.msg.Utm,
    #                 handle_turtle_pose)
    rospy.spin()
