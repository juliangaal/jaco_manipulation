#!/usr/bin/env python
import os
import random
#from geometry_msgs.msg import Point

dir_path = os.path.dirname(os.path.realpath(__file__))
file = open(dir_path + "/baseline_poses.csv", "w")
file.write("x,y,z,dim_x,dim_y,dim_z\n")

fake_file = open(dir_path + "/fake_data.csv", "w")
fake_file.write("Time;Current Pose;Target Pose;Result\n")

num_of_pose = 0
valid_pose = True

while (num_of_pose <= 100):
    x = random.uniform(0.25, 0.7)
    y = random.uniform(0.0, 0.58)
    z = random.uniform(0.0, 0.2)	
	
    result = 'success' if bool(random.getrandbits(1)) == True else 'failure'

    #point = Point()
    #point.x = x
    #point.y = y
    #point.z = z

    file.write("{},{},{},{},{},{}\n".format(x, y, z, 0.06, 0.06, 0.06))
    fake_file.write("{};({},{},{})/(0,0,0,1);({},{},{})/(0,0,0,1);{}\n".format('now',x,y,z,x,y,z,result))	
    num_of_pose += 1

fake_file.close()
file.close()
