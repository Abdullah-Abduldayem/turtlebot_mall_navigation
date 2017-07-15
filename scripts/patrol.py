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
@description:
  Class for handling navigation around the mall scenario between fixed waypoints.
  Interrupt handling for "button presses" is included to deviate from the
  fixed path and move to the desired store.
"""

import rospy
import sys
from std_msgs.msg import UInt8
from collections import deque

from navigate import MallNavigation

class Patrol():
    def __init__(self):
        rospy.init_node('patrol_navigation', anonymous=False)

        #######
        ## Variables
        #######
        self.patrol_waypoints = [
          [5, 3.5, 90],
          [5, 10, 0],
          [15, 10, -90],
          [15, 3.5, 180],
          ]

        self.shop_waypoints = [
          [ 1.5,  2.5, 180], #1
          [ 1.5,  8.5, 180], #2
          [ 6.0, 13.0,  90], #3
          [12.5, 13.0,  90], #4
          [18.0, 10.0,   0], #5
          [18.0,  6.5,   0], #6
          [18.0,  3.5,   0], #7
          [15.0,  1.0, -90], #8
          [ 9.5,  1.0, -90], #9
          [13.0,  7.0, 180], #10
          [ 8.0,  8.0,   0], #11
          [ 8.0,  6.0,   0], #12
          ]

        self.shop_queue = deque()
        self.is_queue_done = True
        self.patrol_idx = None

        # Initialize navigation class
        self.nav = MallNavigation()

        #######
        ## Topic handlers
        #######
        rospy.Subscriber("StoreButton", UInt8, self.buttonCallback)

        #######
        ## Start
        #######
        self.run()


    def run(self):
        self.findClosestPatrolWaypoint()

        while not rospy.is_shutdown():
            # Check if the button was pressed. If so, process it.
            if (len(self.shop_queue) > 0):
                store_index = self.shop_queue.popleft()
                print("Navigating to store " + str(store_index))

                # Get waypoint to desired store
                w = self.shop_waypoints[store_index-1]

                # Navigate to store
                self.nav.moveToGoal(w[0], w[1], w[2])

                # Set the next waypoint if this is the last store in queue
                self.findClosestPatrolWaypoint()

            else:
                self.is_queue_done = True

                w = self.patrol_waypoints[ self.patrol_idx ]
                self.nav.moveToGoal(w[0], w[1], w[2])

                # Set to the next waypoint
                self.patrol_idx = (self.patrol_idx + 1) % len(self.patrol_waypoints)


    def findClosestPatrolWaypoint(self):
        idx = 0
        min_dist = None

        for i in range(len(self.patrol_waypoints)):
            w = self.patrol_waypoints[i]
            dist = self.nav.getDistanceToWaypoint(w)

            if (min_dist == None or min_dist > dist):
                min_dist = dist
                idx = i

        self.patrol_idx = idx

    def buttonCallback(self, msg):
        ## NOTE: This callback blocks execution of the main patrol() function
        ## After completing this callback, set the robot to the closest patrol waypoint

        # Check store number is valid
        store = msg.data
        if (store > 12 or store < 1):
            print("Invalid store number recieved. Ignoring.")
            return

        # Queue up the store number
        self.shop_queue.append(store)
        print("Button " + str(store) + " pressed. Queuing...")

        # If we just inserted the first entry in the queue, cancel the current patrol
        # This immediately starts navigation to the shop
        if (self.is_queue_done):
            self.is_queue_done = False
            self.nav.cancelGoal()

if __name__=='__main__':
    # Begin patrolling
    try:
        Patrol()
        #rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Terminating navigation node.")
