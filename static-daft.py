#!/usr/bin/env python

from __future__ import print_function

import rospy

import random
import math

from dronelib import Drone
import util

from ascend_msgs.srv import GlobalMap
from geometry_msgs.msg import Pose, PoseArray


def goal_callback(msg):
    global goal
    goal = msg.position

def dynamic_obstacles_callback(msg):
    global obstacles
    obstacles = msg.poses

def boost_callback(msg):
    global boosts
    boosts = msg.poses

def main():
    # Init ROS node
    rospy.init_node('task', anonymous=True)

    # Create subscriber for position, goal, boost points, and obstacles
    rospy.Subscriber('/goal', Pose, goal_callback)
    rospy.Subscriber('/boost', PoseArray, boost_callback)
    rospy.Subscriber("/dynamic_obstacles", PoseArray, dynamic_obstacles_callback)

    # Wait for resources to become active
    goal = rospy.wait_for_message("/goal", Pose).position
    boosts = rospy.wait_for_message("/boost", PoseArray).poses
    obstacles = rospy.wait_for_message("/dynamic_obstacles", PoseArray).poses

    # Create map service client
    getMap = rospy.ServiceProxy('/GlobalMap', GlobalMap)
    rospy.wait_for_service('/GlobalMap')

    try:
        raw_map = getMap()
    except rospy.ServiceException as e:
        print("Map service error: " + str(e))
        return

    # Get map as 2D list
    world_map = util.parse_map(raw_map)

    # Print resources
    print("Wall layout:")
    util.print_map(world_map)
    print("Boost points:")
    util.print_positions(boosts)
    print("Obstacles at start:")
    util.print_positions(obstacles)

    # Initialize drone
    drone = Drone()
    drone.takeoff()

    # -- For example code --
    target_x = 0
    target_y = 0

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()
        # -------------------------------
        # ------Replace this example-----
        # ---------with your code!-------
        # -------------------------------
        # Find how far we are away from target
        distance_to_target = ((target_x - drone.position.x)**2 +
                              (target_y - drone.position.y)**2)**0.5

        # Do special action if we are close
        if distance_to_target < 0.5:
            # Print current distance to goal. Note that we 
            # wont reach the goal, since we just move randomly
            distance_to_goal = ((drone.position.x - goal.x)**2 +
                                (drone.position.y - goal.y)**2)**0.5

            print("Distance to goal is now", distance_to_goal)

            # Generate some random point and rotation
            target_x = random.randint(-3, 3)
            target_y = random.randint(-3, 3)

            # Move to random point
            drone.set_target(target_x, target_y)
        else:
            print("distance to target:", distance_to_target)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
