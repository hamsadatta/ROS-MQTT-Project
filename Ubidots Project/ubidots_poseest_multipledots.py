#!/usr/bin/env python
# -*- coding: utf-8 -*-

import paho.mqtt.client as mqttClient
import time
import json
import random
#this is a simple program that subscribes to the odom topic and gets the position and orientation of the robot
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
from std_srvs.srv import Empty
import tf


'''
global variables
'''
connected = False  # Stores the connection status
BROKER_ENDPOINT = "things.ubidots.com"
PORT = 1883
MQTT_USERNAME = "BBFF-6H4Gbde0NsmXBnephyfVH87ve5uGnD"  # Put your TOKEN here
MQTT_PASSWORD = "0033"
TOPIC = "/v1.6/devices/ROBOT"
DEVICE_LABEL = "_LOCATOR"
VARIABLE_LABEL_1 = "xdata"
VARIABLE_LABEL_2 = "ydata"
VARIABLE_LABEL_3 = "yawdata"

'''
Functions to process incoming and outgoing streaming
'''


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[INFO] Connected to broker")
        global connected  # Use global variable
        connected = True  # Signal connection

    else:
        print("[INFO] Error, connection failed")


def on_publish(client, userdata, result):
    print("[INFO] Published!")


def connect(mqtt_client, mqtt_username, mqtt_password, broker_endpoint, port):
    global connected

    if not connected:
        mqtt_client.username_pw_set(mqtt_username, password=mqtt_password)
        mqtt_client.on_connect = on_connect
        mqtt_client.on_publish = on_publish
        mqtt_client.connect(broker_endpoint, port=port)
        mqtt_client.loop_start()

        attempts = 0

        while not connected and attempts < 5:  # Waits for connection
            print("[INFO] Attempting to connect...")
            time.sleep(1)
            attempts += 1

    if not connected:
        print("[ERROR] Could not connect to broker")
        return False

    return True


def publish(mqtt_client, topic, payload):
    try:
        mqtt_client.publish(topic, payload)
    except Exception as e:
        print("[ERROR] There was an error, details: \n{}".format(e))

def odomPoseCallback(odom_msg):

    #print "odom pose callback"
    #get the position of the robot
    print "POSITION OF ROBOT IS:"
    print 'x = ',odom_msg.pose.pose.position.x
    print 'y = ', odom_msg.pose.pose.position.y 
    print 'z = ', odom_msg.pose.pose.position.z
    
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

    yaw1 = math.degrees(yaw)
    payload1 ='"xd": {"value": '+str(odom_msg.pose.pose.position.x)+', "context":{"name":'+'"'+"yaw of the robot is : "+str(odom_msg.pose.pose.position.x)+'"'+'}}'
    payload2 ='"yd": {"value": '+str(odom_msg.pose.pose.position.y)+', "context":{"name":'+'"'+"yaw of the robot is : "+str(odom_msg.pose.pose.position.y)+'"'+'}}'
    payload ='{"yawd": {"value": '+str(yaw1)+', "context":{"name":'+'"'+"yaw of the robot is : "+str(yaw1)+'"'+'}},'+payload1+','+payload2+'}'
    
    topic = "{}{}".format(TOPIC, DEVICE_LABEL)

    if not connected:  # Connects to the broker
        connect(mqtt_client, MQTT_USERNAME, MQTT_PASSWORD,
                BROKER_ENDPOINT, PORT)

    # Publishes values
    print("[INFO] Attempting to publish payload:")
    print(payload)
    publish(mqtt_client, topic, payload)


if __name__ == '__main__':
  
  mqtt_client = mqttClient.Client()
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
