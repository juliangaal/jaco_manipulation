#!/usr/bin/env python
import os
import random
from geometry_msgs.msg import Point

dir_path = os.path.dirname(os.path.realpath(__file__))
file = open(dir_path + "/baseline_poses.csv", "w")
file.write("x,y,z,dim_x,dim_y,dim_z\n")

num_of_pose = 0
valid_pose = True

while (num_of_pose <= 100):
    x = random.uniform(0.2, 0.7)
    y = random.uniform(0.0, 0.58)
    z = random.uniform(0.0, 0.2)

    point = Point()
    point.x = x
    point.y = y
    point.z = z

    file.write("{},{},{},{},{},{}\n".format(point.x, point.y, point.z, 0.06, 0.06, 0.06))

    num_of_pose += 1

file.close()
