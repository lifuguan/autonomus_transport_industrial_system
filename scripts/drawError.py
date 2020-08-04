'''
Author: lifuguan
Date: 2020-08-04 16:07:21
LastEditTime: 2020-08-04 16:29:58
LastEditors: Please set LastEditors
Description: Aimed to draw the relative position between frame /odom and frame /map
FilePath: /autonomus_transport_industrial_system/scripts/drawError.py
'''
import rospy
from matplotlib import pyplot
import tf

rospy.init_node("drawingError")

listener = tf.TransformListener()

rate = rospy.Rate(10.0)
while not rospy.is_shutdown():
    try:
        (trans, rot) = listener.lookupTransform('odom', 'map', rospy.Time(0))
        print(trans)
    except (tf.LookupException, tf.ConnectivityException):
        continue