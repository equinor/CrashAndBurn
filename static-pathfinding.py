#!/usr/bin/env python

from __future__ import print_function

import rospy

import random
import math

from dronelib import Drone
import util

from ascend_msgs.srv import GlobalMap
from geometry_msgs.msg import Pose, PoseArray

from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder

import reverse

def goal_callback(msg):
    global goal
    goal = msg.position

def dynamic_obstacles_callback(msg):
    global obstacles
    obstacles = msg.poses

def boost_callback(msg):
    global boosts
    boosts = msg.poses

def putBoostToMap(world_map, boosts):
    for boost in boosts:
        world_map[boost.y, boost.x] = 1

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
    world_map_boosted = putBoostToMap(world_map, boosts)

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

    x_end = 6
    y_end = 19

    coords = create_waypoints(world_map, y_end, x_end)
    print(coords)
    i=0

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        rate.sleep()
        # -------------------------------
        # ------Replace this example-----
        # ---------with your code!-------
        # -------------------------------
        # Find how far we are away from target
        if i == 0:
            drone.set_target(0, 0)
        distance_to_target = ((target_x - drone.position.x)**2 +
                             (target_y - drone.position.y)**2)**0.5

        # Do special action if we are close
        if distance_to_target < 0.25:

            if i < len(coords)-1:
                i = i + 1
                print("Next index is ", i)
            else:
                drone.takeoff(height=0)

            target_x = coords[i][1]
            target_y = coords[i][0]

            # Move to random point
            drone.set_target(target_x, target_y)

        else:
            print("distance to target:", distance_to_target)


def create_waypoints(matrix, y_end, x_end):
    coords = [[0, 0],
              [3.5, 2.5],
              [3.5, 10.5],
              [3.5, 22.5],
              [10.5, 22.5],
              [20, 22.5],
              [34.5, 22.5],
              [34.5, 12],
              [34.5, 2.5],
              [19.5, 2.5],
              [19.5, 10.5]
              ]

    reversed_test_maze = reverse.reverseZeroOne(matrix)
    grid = Grid(matrix=reversed_test_maze)

    start = grid.node(0, 0)
    end = grid.node(y_end, x_end)

    finder = AStarFinder(diagonal_movement=DiagonalMovement.never)
    path, runs = finder.find_path(start, end, grid)

    print(grid.grid_str(path=path, start=start, end=end))

    for i in range(len(path)):
        (x, y) = path[i]
        path[i] = (y + 0.5, x + 0.5)

    return path


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

