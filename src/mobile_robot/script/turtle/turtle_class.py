#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2018 Cesar Sinchiguano <cesarsinchiguano@hotmail.es>
#
# Distributed under terms of the BSD license.

"""

"""

from turtle_library import *


####################################################
#==================================================
class Turtlebot(object):
    def __init__(self, robotID=None):
        if (robotID is None or robotID<=0):
            print("No valid ID given! Using ID:1 by default")
            robotID = 1
        # subscribe topics
        self.sub_odom = rospy.Subscriber('/robot'+str(robotID)+'/odom', Odometry, self.odom_callback)
        self.sub_globPos = rospy.Subscriber('/vrpn_client_node/turtle'+str(robotID)+'/pose', PoseStamped, self.globPos_callback)
        # publish topics
        self.pub_cmd_vel = rospy.Publisher('/robot'+str(robotID)+'/mobile_base/commands/velocity', Twist, queue_size=1)
        self.pub_reset_odometry = rospy.Publisher('/robot'+str(robotID)+'/mobile_base/commands/reset_odometry', Empty, queue_size=1)
        #self.pub_sound = rospy.Publisher('/mobile_base/commands/sound', Sound, queue_size=1)

        # variables
        self.odometry_sensor=0.0
        self.globPos_sensor=0.0

    def reset_odometry(self):
        # reset odometry (these messages take a few iterations to get through)
        startingTime = time.time()
        while ((time.time() - startingTime)< 0.25):
            self.pub_reset_odometry.publish(Empty())

    def cmd_velocity(self, linear=0, angular=0):

        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.pub_cmd_vel.publish(msg)
    #
    # def play_sound(self, sound_id=0):
    #     msg = Sound()
    #     msg.value = max(0, min(sound_id, 6))
    #     self.pub_sound.publish(msg)

    def odom_callback(self,msg):
        self.odometry_sensor=msg

    def globPos_callback(self,msg):
        self.globPos_sensor=msg

    def get_globPos(self):
        if self.globPos_sensor:
            tmp = np.zeros((3))
            tmp[0] = self.globPos_sensor.pose.position.x
            tmp[1] = self.globPos_sensor.pose.position.y

            r, p, y = tf.transformations.euler_from_quaternion([
                self.globPos_sensor.pose.orientation.x,
                self.globPos_sensor.pose.orientation.y,
                self.globPos_sensor.pose.orientation.z,
                self.globPos_sensor.pose.orientation.w])

            tmp[2] = y
            return tmp
        else:
            return None

    def get_odometry(self):
        if self.odometry_sensor:
            tmp = np.zeros((3))
            tmp[0] = self.odometry_sensor.pose.pose.position.x
            tmp[1] = self.odometry_sensor.pose.pose.position.y

            r, p, y = tf.transformations.euler_from_quaternion([
                self.odometry_sensor.pose.pose.orientation.x,
                self.odometry_sensor.pose.pose.orientation.y,
                self.odometry_sensor.pose.pose.orientation.z,
                self.odometry_sensor.pose.pose.orientation.w])

            tmp[2] = y
            return tmp
        else:
            return None

    def is_shutting_down(self):
        return rospy.is_shutdown()
