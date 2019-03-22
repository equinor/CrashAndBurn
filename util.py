#!/usr/bin/env python
from __future__ import print_function


def parse_map(msg):
    num_rows = msg.dim[0].size
    num_cols = msg.dim[1].size
    world_map = bytearray(msg.data)
    world_map = [[world_map[num_cols*y + x] for x in range(num_cols)] for y in range(num_rows)]
    return world_map

def print_map(world_map):
    for y in range(len(world_map)):
        for x in range(len(world_map[y])):
            print(world_map[y][x], sep=' ', end='')
        print()

def print_positions(pose_array):
    for pose in pose_array:
        pos = (pose.position.x, pose.position.y, pose.position.z)
        print(pos, end=", ")
    print()
