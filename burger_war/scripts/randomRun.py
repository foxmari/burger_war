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
from sensor_msgs.msg import JointState

import tf

PI = 3.1416
# 8x8  [rad]
TARGET_TH = (
    (-PI / 4, -PI / 4, -PI / 2, -PI / 2, -PI * 3 / 4, -PI * 3 / 4, -PI * 3 / 4, -PI * 3 / 4),
    (-PI / 4, -PI / 4, -PI / 4, -PI / 4, -PI * 3 / 4, -PI * 3 / 4, -PI * 3 / 4, -PI * 3 / 4),
    (-PI / 4, -PI / 4, -PI / 4, 0, -PI / 2, -PI * 3 / 4, -PI * 3 / 4, PI),
    (-PI / 4, -PI / 4, 0, 0, -PI / 2, -PI / 2, -PI * 3 / 4, PI),
    (0, PI / 4, PI / 2, PI / 2, PI, PI, PI * 3 / 4, PI * 3 / 4),
    (0, PI / 4, PI / 3, PI / 2, PI, PI * 3 / 4, PI * 3 / 4, PI * 3 / 4),
    (PI / 4, PI / 4, PI / 3, PI / 4, PI * 3 / 4, PI * 3 / 4, PI * 3 / 4, PI * 3 / 4),
    (PI / 4, PI / 4, PI / 4, PI / 4, PI / 2, PI / 2, PI * 3 / 4, PI * 3 / 4),
)
WIDTH = 1.2 * (2 ** 0.5)  # [m]


class TeriyakiBurger():
    def __init__(self, bot_name):
        # bot name
        self.name = bot_name
        # robot state 'inner' or 'outer'
        self.state = 'outer'
        self.innerstate = 'turn'
        # robot wheel rot
        self.wheel_rot_r = 0
        self.wheel_rot_l = 0
        self.now_wheel_rot_l = 0
        self.now_add_wheel_rot_l = 0
        self.pose_x = 0
        self.pose_y = 0

        self.k = 0.5

        # speed [m/s]
        self.speed = 0.07

        # publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # subscriber
        self.pose_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.poseCallback)
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)
        self.odom_sub = rospy.Subscriber('joint_states', JointState, self.jointstateCallback)

        self.twist = Twist()
        self.twist.linear.x = self.speed;
        self.twist.linear.y = 0.;
        self.twist.linear.z = 0.
        self.twist.angular.x = 0.;
        self.twist.angular.y = 0.;
        self.twist.angular.z = 0.

    def poseCallback(self, data):
        '''
        pose topic from amcl localizer
        update robot twist
        '''
        pose_x = data.pose.pose.position.x
        pose_y = data.pose.pose.position.y
        self.pose_x =pose_x
        self.pose_y = pose_y
        quaternion = data.pose.pose.orientation
        rpy = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        th = rpy[2]

        th_xy = self.calcTargetTheta(pose_x, pose_y)

        th_diff = th_xy - th
        while not PI >= th_diff >= -PI:
            if th_diff > 0:
                th_diff -= 2 * PI
            elif th_diff < 0:
                th_diff += 2 * PI
        new_twist_ang_z = th_diff * self.k

        self.twist.angular.z = new_twist_ang_z

        print(pose_x, pose_y, th, th_xy, new_twist_ang_z)

    def calcTargetTheta(self, pose_x, pose_y):
        x = self.poseToindex(pose_x)
        y = self.poseToindex(pose_y)
        th = TARGET_TH[x][y]
        print("POSE pose_x: {}, pose_y: {}. INDEX x:{}, y:{}".format(pose_x, pose_y, x, y))
        return th

    def poseToindex(self, pose):
        i = 7 - int((pose + WIDTH) / (2 * WIDTH) * 8)
        i = max(0, min(7, i))
        return i

    def lidarCallback(self, data):
        '''
        lidar scan use for bumper
        controll speed.x
        '''
        is_near_wall = self.isNearWall(data.ranges)
        if is_near_wall:
            self.twist.linear.x = -self.speed
        else:
            self.twist.linear.x = self.speed

    def isNearWall(self, scan):
        if not len(scan) == 360:
            return False
        forword_scan = scan[:10] + scan[-10:]
        # drop too small value ex) 0.0
        forword_scan = [x for x in forword_scan if x > 0.1]
        if min(forword_scan) < 0.2:
            return True
        return False

    def innerTwist(self):
        '''
               calc twist from self.state
               'go' -> self.speed,  'back' -> -self.speed
        '''
        z = 0
        x = 0
        if self.innerstate == 'turn':
            z = -0.4
            x = 0
        elif self.innerstate == 'go':
            # set speed x axis
            z = 0
            x = self.speed
        elif self.innerstate == 'back':
            # set speed x axis
            z = -0.08
            x = -1 * self.speed
        elif self.innerstate == 'endturn':
            # set speed x axis
            z = 0.4
            x = 0
        else:
            # error state
            x = 0
            rospy.logerr("SioBot state is invalid value %s", self.state)

        twist = Twist()
        twist.linear.x = x;
        twist.linear.y = 0;
        twist.linear.z = 0
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = z
        return twist

    def jointstateCallback(self, data):
        '''
        update wheel rotation num
        '''
        self.wheel_rot_r = data.position[0]
        self.wheel_rot_l = data.position[1]

    def calcInnerState(self):
        '''
        update robot state 'go' or 'back'
        '''
        # if self.innerstate != 'go' and abs(self.now_add_wheel_rot_l) < 7:
        #     self.innerstate = 'turn'
        if self.innerstate == 'turn' and abs(self.now_add_wheel_rot_l) > 5:
            self.innerstate = 'go'
        elif self.innerstate == 'go' and abs(self.now_add_wheel_rot_l) > 30:
            self.innerstate = 'back'
        elif self.innerstate == 'back' and abs(self.now_add_wheel_rot_l) < 5:
            self.innerstate = 'endturn'
        elif self.innerstate == 'endturn' and abs(self.now_add_wheel_rot_l) < 2:
            self.innerstate = 'end'
            self.state = 'outer'

    def calcTwist(self):
        '''
        calc twist from self.state
        'go' -> self.speed,  'back' -> -self.speed
        '''
        if self.state == 'inner':
            twist = self.innerTwist()
            print("mode inner!!!!!!!ni haitta")
            print(self.now_add_wheel_rot_l)
            print(self.wheel_rot_l, self.now_wheel_rot_l)
            print(self.innerstate)
            print

        elif self.innerstate =='end':
            self.state = 'outer'
            twist = self.twist
            if 0.2 > abs(self.pose_y) :
                self.innerstate == "turn"

        else:
            # set speed x axis
            twist = self.twist
            if 0.2 > abs(self.pose_y) :
                self.innerstate == "turn"
        return twist

    def calcState(self):
        '''
        update robot state `sicle` or 'run & back'
        '''
        y = self.pose_y
        if self.innerstate =='turn' and 2 > abs(y) > 1.35:
            self.state = 'inner'
        elif self.state=='outer':
            self.state = 'outer'
            self.now_wheel_rot_l = self.wheel_rot_l
        # elif self.state=='outer'
        #     self.state = 'outer'
        #     self.now_wheel_rot_l = self.wheel_rot_l

    def strategy(self):
        '''
        calc Twist and publish cmd_vel topic
        Go and Back loop forever
        '''
        r = rospy.Rate(10)  # change speed 10fps

        while not rospy.is_shutdown():
            # update state from now state and wheel rotation
            self.calcState()
            self.calcInnerState()
            self.now_add_wheel_rot_l = abs(self.wheel_rot_l) - abs(self.now_wheel_rot_l)
            print(self.now_add_wheel_rot_l ,abs(self.wheel_rot_l) ,abs(self.now_wheel_rot_l),self.innerstate)

            ## update twist
            twist = self.calcTwist()

            ## publish twist topic
            self.vel_pub.publish(twist)

            r.sleep()



if __name__ == '__main__':
    rospy.init_node('random_rulo')
    bot = TeriyakiBurger('mari_burger')
    bot.strategy()

