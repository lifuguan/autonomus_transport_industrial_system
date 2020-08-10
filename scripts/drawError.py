#!/usr/bin/env python
'''
Author: lifuguan
Date: 2020-08-04 16:07:21
LastEditTime: 2020-08-09 17:42:00
LastEditors: Please set LastEditors
Description: Aimed to draw the relative position between frame /odom and frame /map
FilePath: /autonomus_transport_industrial_system/scripts/drawError.py
'''
import rospy
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
import numpy as np
import time
import math


class DrawError(object):
    odom_time, cb_time = 0, 0
    x_odom = []
    y_odom = []
    x_rf2o = []
    y_rf2o = []

    def __init__(self):
        pass

    def isCBAlive(self):
        return False if self.cb_time - self.odom_time > 2 else True

    def odomCallBack(self, data):
        self.x_odom.append(data.pose.pose.position.x)
        self.y_odom.append(data.pose.pose.position.y)

    def rf2oCallBack(self, data):
        self.x_rf2o.append(data.pose.pose.position.x)
        self.y_rf2o.append(data.pose.pose.position.y)
        self.odom_time = time.time()

    # check if data is not received. If so, exit and plot the figure
    def cbMonitor(self, plt):
        self.cb_time = time.time()
        if not self.isCBAlive():
            rospy.logwarn("No odom data received!Exit!")
            return True
        else:
            return False


if __name__ == "__main__":
    rospy.init_node("drawingError")
    de = DrawError()

    # gazebo's odom, without nosie
    rospy.Subscriber("odom", Odometry, de.odomCallBack)
    # rf2o_odometry's odom
    rospy.Subscriber("odom_rf2o", Odometry, de.rf2oCallBack)
    rospy.sleep(2)
    rospy.Timer(rospy.Duration(3), de.cbMonitor)

    while not rospy.is_shutdown():
        if de.cbMonitor(plt) == True:
            break
    ax1 = plt.subplot(1, 2, 1)
    plt.title("odom display")
    plt.plot(de.x_odom, de.y_odom, color="b", label="/odom")
    plt.plot(de.x_rf2o, de.y_rf2o, color="r", label="/odom_rf2o")
    plt.legend()

    # down-sampling for gazebo's /odom
    rate = len(de.y_odom) / len(de.y_rf2o)
    print rate, len(de.x_rf2o)
    ax2 = plt.subplot(1, 2, 2)
    errs = []
    for i in range(len(de.x_rf2o)):
        print i
        errs.append(
            math.hypot(de.x_odom[i * rate] - de.x_rf2o[i],
                       de.y_odom[i * rate] - de.y_rf2o[i]))
    t_ = list(range(0, len(errs)))
    print len(errs)
    plt.plot(t_,
             errs,
             color="g",
             linewidth=1,
             linestyle=':',
             label='Only rf2o errors')

    plt.legend()
    plt.show()
