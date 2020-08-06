#!/usr/bin/env python
'''
Author: lifuguan
Date: 2020-08-04 16:07:21
LastEditTime: 2020-08-06 18:18:30
LastEditors: Please set LastEditors
Description: Aimed to draw the relative position between frame /odom and frame /map
FilePath: /autonomus_transport_industrial_system/scripts/drawError.py
'''
import rospy
from matplotlib import pyplot
from nav_msgs.msg import Odometry
import time

pyplot.ion()
pyplot.title("odom display")
pyplot.grid(True)


class DrawError(object):
    self.odom_time, self.cb_time = 0, 0
    self.locations = []
    def isAlive(self):
        return False if cb_time - odom_time > 2 else True
    
    def odomCallBack(self, data):
        locations.append([data.pose.pose.position.x, data.pose.pose.position.y])
        odom_time = time.time()

    def process(self, event):
        cb_time = time.time()
        if not self.isAlive():
            print "No data received!Exit!"


if __name__ == "__main__":
    rospy.init_node("drawingError")
    de = DrawError()
    rospy.Subscriber("odom", Odometry, de.odomCallBack)
    rospy.Timer(rospy.Duration(0.05), de.process)
    rospy.spin()


