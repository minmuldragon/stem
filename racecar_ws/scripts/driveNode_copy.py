#!/usr/bin/env python

import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan
from racecar_ws.msg import drive_msg
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray


AUTONOMOUS_MODE=False
count = 0
pi=3.141592
print("Complete to open Drive Node")

class Drive:
	def __init__(self):
		rospy.init_node("potentialField")
		self.data = None
		self.cmd = drive_msg()
                self.camera_sub = rospy.Subscriber("/camera", Image, self.camera_callback)
		self.ML_sub = rospy.Subscriber("/teachable_machine", Float32MultiArray, self.machine_learning_callback)
		self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback) 
		self.drive_pub = rospy.Publisher("/drive", drive_msg, queue_size=1)

		self.cartPoints = [None for x in range(500)]
		self.finalVector = [100, 0]
		self.max= 255

	
	def scan_callback(self, data):
		'''Checks LIDAR data'''
                inf = 6
                sum_x=0
                sum_y=0
		self.data = data.ranges
                for i in range(500):
                    if(self.data[i]>=6 or self.data[i]==inf):
                        self.data[i]=6
                    elif(self.data[i]==0.0):
                        continue
                    sum_x +=math.cos(float(i)/500*2*pi)*self.data[i]
                    sum_y +=math.sin(float(i)/500*w*pi)*self.data[i]
                self.cmd.drive_angle= k* math.arctan(sum_y/sum_x)
            

                self.drive_callback()
		
        def camera_callback(self, msg):
		'''camera_callback'''

	def machine_learning_callback(self, msg):
		'''machine_learning_callback'''

	def drive_callback(self):
		'''Publishes drive commands'''
                self.cmd.velocity=100
		self.drive_pub.publish(self.cmd)
                


if __name__ == "__main__":
	try:
		node = Drive()
		rospy.spin()		
	except rospy.ROSInterruptException:
		exit()
