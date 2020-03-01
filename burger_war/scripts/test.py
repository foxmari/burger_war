#!/usr/bin/env python
# -*- coding: utf-8 -*-
'''
This is rumdom run node.
subscribe No topcs.
Publish 'cmd_vel' topic.
mainly use for simple sample program

by Takuya Yamaguhi.
'''

import rospy
import random

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

import tf


pose_x = odom.pose.pose.position.x
pose_y = odom.pose.pose.position.y
quaternion = odom.pose.pose.orientation
rpy = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
th = rpy[2]

th_xy = calcTargetTheta(pose_x, pose_y)

th_diff = th_xy - th
while not PI >= th_diff >= -PI:
    if th_diff > 0:
        th_diff -= 2 * PI
    elif th_diff < 0:
        th_diff += 2 * PI
new_twist_ang_z = th_diff * self.k

twist.angular.z = new_twist_ang_z
print(pose_x, pose_y, th, th_xy, new_twist_ang_z)