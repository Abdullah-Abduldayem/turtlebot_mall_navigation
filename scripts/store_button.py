#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 13/07/2017
@author: Abdullah Abduldayem

Publish the number of the shop that pressed the virtual button
"""

import rospy
from std_msgs.msg import UInt8
import sys



def publishButton(store):
    pub = rospy.Publisher('StoreButton', UInt8, queue_size=10)
    rospy.init_node('store_button', anonymous=True)
    pub.publish(store)


def helpMsg():
  print("\nPlease input store number (1-12)")
  print("rosrun turtlebot_mall_navigation store_button.py <num>\n")


def main():
    # Check if a store number was entered
    if (len(sys.argv) < 2):
        helpMsg()
        return

    # Get store number from user input
    store = float(sys.argv[1])

    ## Check if store is an int between 1 to 12
    if (not store.is_integer()
    or store > 12
    or store < 1):
        helpMsg()
        return

    # Publish button number
    publishButton(store)


if __name__ == '__main__':
    main()
