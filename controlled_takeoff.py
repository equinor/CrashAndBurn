#!/usr/bin/env python
"""Controlling and monitoring drone takeoff

This code demonstrates how to control the takeoff height 
and how to monitor progress during takeoff. 

How to run:
    Make sure the simulator is running, and run the following 
    in a separate terminal window
    $ rosrun techathon controlled_takeoff.py
"""

from __future__ import print_function

import time
import random

import rospy
from dronelib import Drone

def main():
    rospy.init_node("dronelib_example")

    # Create drone, activate it and send takeoff command
    drone = Drone()

    # Takeoff and override default takeoff height (3 meters)
    drone.takeoff(height=2.0)

    # Monitor takeoff and dont progress until we are high enough
    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        # Control rate of while loop
        rate.sleep()
        
        # Monitor height
        if drone.position.z > 1.5:
            break

    print("Takeoff complete")

    ## Takeoff completed, do something useful
    # Generate some random point
    x = random.randint(-3, 3)
    y = random.randint(-3, 3)

    # Move to random point
    drone.set_target(x, y)


if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
