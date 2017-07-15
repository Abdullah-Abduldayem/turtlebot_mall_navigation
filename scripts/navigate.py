#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Copyright 2017 Abdullah Abduldayem

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

@author: Abdullah Abduldayem
@created: 13/07/2017
@description: A class for handling turtlebot waypoint navigation using actionlib
"""

import rospy
from math import radians, degrees, cos, sin, tan, pi, atan2, sqrt

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *

from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, PoseWithCovariance, Twist, Point, Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class MallNavigation():
    def __init__(self):
        ############
        ## Variables
        ############
        self.current_pose = [None, None, None]

        ############
        ## Topic Handlers
        ############
        rospy.Subscriber("base_pose_ground_truth", Odometry, self.odomCallback)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        ## Allow up to 5 seconds for the action server to come up
        self.move_base.wait_for_server(rospy.Duration(5))


        ## Send a blank goal and cancel it to help start things
        goal = MoveBaseGoal()
        self.move_base.send_goal(goal)
        success = self.move_base.wait_for_result(rospy.Duration(1.0))
        self.move_base.cancel_goal()

    def cancelGoal(self):
        print("Cancelling goal")
        self.move_base.cancel_goal()


    def generateRelativePositionGoal(self, pose):
        ## Compute pose
        x = pose[0]
        y = pose[1]
        z = 0
        yaw = pose[2]*pi/180
        quat = quaternion_from_euler(0.0, 0.0, yaw)

        return Pose( Point(x, y, z) , Quaternion(*quat.tolist()) )


    def getDistanceToWaypoint(self, waypoint):
        # If odom hasn't returned the current position, wait for it
        while (self.current_pose[0] == None and not rospy.is_shutdown()):
          rate = rospy.Rate(1)
          rate.sleep()

        x = self.current_pose[0] - waypoint[0]
        y = self.current_pose[1] - waypoint[1]
        return sqrt(x*x + y*y)


    def moveToGoal(self, x, y, yaw_deg):
        ## Cancel previous goals
        self.move_base.cancel_goal()

        ## Generate new pose
        pose = [x, y, yaw_deg]

        # Move to goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.generateRelativePositionGoal(pose)
        rospy.loginfo("Recieved new waypoint. Navigating...")

        #start moving
        self.move_base.send_goal(goal)

        #allow TurtleBot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(120))

        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("Navigation not complete after 120 seconds. Cancelling waypoint...")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Navigation successfully completed.")


    def odomCallback(self, msg):
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y
        self.current_pose[2] = 0 #Yaw
