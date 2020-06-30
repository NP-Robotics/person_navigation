#!/usr/bin/env python3


import rospy
import cv2
import time
import argparse
import numpy as np

from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float32, Bool, Int32MultiArray, Int32
from person_navigation.msg import Polar

class PersonNavigation(object):
    def __init__(self, args):

        self.args = args

        if args.target_polar_topic:
            target_polar_topic = args.target_polar_topic
            self.target_depth_sub = rospy.Subscriber(target_polar_topic, Polar, self.target_depth_callback, queue_size=1)
            rospy.loginfo("Subscribing to: " + target_polar_topic)
        else:
            rospy.loginfo("No topic stated. Please check your launch file argument")
            
        #initialize subscriber and publisher
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.nav_params = rospy.get_param("/person_navigation")

        self.args = args

    #convert compressed image to opencv Mat and store opencv Mat in variable
    def target_depth_callback(self, msg):
        angle = msg.angle * 3.14159/180
        depth = msg.depth

        angle_vel = self.calculate_angle_vel(angle)
        linear_vel = self.calculate_linear_vel(depth)

        cmd_vel_msg = Twist()
        cmd_vel_msg.angular.z = angle_vel
        cmd_vel_msg.linear.x = linear_vel

        #publish cmd_vel
        self.cmd_vel_pub.publish(cmd_vel_msg)

        rospy.loginfo("angle velocity: " + str(round(angle_vel, 2)) + " linear velocity: " + str(round(linear_vel, 2)))

    #main callback function for code
    def calculate_angle_vel(self, angle):
        angle_vel = self.nav_params["angle_P"] * angle
        return angle_vel

    def calculate_linear_vel(self, depth):
        target_offset = self.nav_params["target_offset"]
        movement_vel = self.nav_params["movement_P"] * (depth-target_offset)
        return movement_vel

#TODO Setup args 
def parse_args():
    
    parser = argparse.ArgumentParser()
    parser.add_argument("--target_polar_topic", type=str)

    return parser.parse_known_args()

def main(args):
    '''Initializes and cleanup ros node'''
    person_nav = PersonNavigation(args)
    rospy.init_node('person_navigation', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down ROS person navigation")

if __name__ == "__main__":
    args, unknown = parse_args()
    main(args)

    
    