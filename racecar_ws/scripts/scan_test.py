#!/usr/bin/python
# license removed for brevity

import rospy
import time

from racecar_ws.msg import drive_msg
from sensor_msgs.msg import LaserScan

mux_mode = ''



def scan_callback(msg):
    for i in range(500):
        print(str(float(i)/500*360)+": "+str(msg.ranges[i]))
        #init ROS
rospy.init_node('scan_test')
rospy.Subscriber('/scan', LaserScan, scan_callback)


rospy.spin()
