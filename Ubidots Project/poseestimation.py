#!/usr/bin/env python

#this is a simple program that subscribes to the odom topic and gets the position and orientation of the robot
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
from std_srvs.srv import Empty
import tf
from rospy_message_converter import json_message_converter
from std_msgs.msg import String

#callback function the odom pose (position+orientation) of the robot 
def odomPoseCallback(odom_msg):

    #print "odom pose callback"
    #get the position of the robot
    print "POSITION OF ROBOT IS:"
    print 'x = ',odom_msg.pose.pose.position.x
    print 'y = ', odom_msg.pose.pose.position.y 
    print 'z = ', odom_msg.pose.pose.position.z
    
    #get the velocity of the robot
    #print 'vx = ',odom_msg.twist.twist.linear.x
    #print 'vz = ',odom_msg.twist.twist.angular.z
    #print the values of the orientation in quaternion
    #print 'qx=',odom_msg.pose.pose.orientation.x
    #print 'qy=',odom_msg.pose.pose.orientation.y
    #print 'qz=',odom_msg.pose.pose.orientation.z
    #print 'qw=',odom_msg.pose.pose.orientation.w
    
    #formulate a quaternion as a list
    quaternion = (
    odom_msg.pose.pose.orientation.x,
    odom_msg.pose.pose.orientation.y,
    odom_msg.pose.pose.orientation.z,
    odom_msg.pose.pose.orientation.w)
    
    #convert the quaternion to roll-pitch-yaw
    rpy = tf.transformations.euler_from_quaternion(quaternion)
    #extract the values of roll, pitch and yaw from the array
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]

    #print the roll, pitch and yaw
    print "ORIENTATION OF ROBOT IS:"
    print 'roll = ',math.degrees(roll) 
    print 'pitch = ',math.degrees(pitch)
    print 'yaw = ',math.degrees(yaw)

    message1 = String(yaw)
    json_str = json_message_converter.convert_ros_message_to_json(message1)
    print 'yawjson = ',json_str


if __name__ == '__main__':
    try:
        #init the node
        rospy.init_node('kbot_pose', anonymous=True)
       
       #subscribe to the odom topic 
        position_topic = "/odom"
        pose_subscriber = rospy.Subscriber(position_topic, Odometry, odomPoseCallback) 
        #spin
        rospy.spin()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
