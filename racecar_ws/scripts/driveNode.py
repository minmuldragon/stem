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
inf = 6
pi=3.141592
k =20
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
                sum_x=0
                sum_y=0
		self.data = data.ranges
                self.data = list(self.data)
                for i in range(500):
                    if(self.data[i]>=6 or self.data[i]==inf):
                        self.data[i]=6
                    elif(self.data[i]==0.0 or (i>=150 and i<=300)):
                        continue
                    
                    elif ((i>=0 and i<=49) or (i<=499 and i>=450)):
                        self.data[i]*=0.7
                    sum_x +=math.cos(float(i)/500*2*pi)*self.data[i]
                    sum_y +=math.sin(float(i)/500*2*pi)*self.data[i]
                
                a= math.atan(sum_y/sum_x)/3.141592*180
                if sum_y<0 and a>33: self.cmd.drive_angle= -245
                if sum_y>0 and a>33: self.cmd.drive_angle= 245
            

                self.drive_callback()
		
        def camera_callback(self, msg):
		'''camera_callback'''

	def machine_learning_callback(self, msg):
		'''machine_learning_callback'''

	def drive_callback(self):
		'''Publishes drive commands'''
                self.cmd.velocity=255
		self.drive_pub.publish(self.cmd)
                


if __name__ == "__main__":
	try:
		node = Drive()
		rospy.spin()		
	except rospy.ROSInterruptException:
		exit()
