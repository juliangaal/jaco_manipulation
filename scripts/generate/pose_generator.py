#!/usr/bin/env python
import os
import random


class PoseGenerator:
    def __init__(self, file_path, total_poses):
        self.parent_dir = os.path.dirname(os.getcwd())
        self.file_path = self.parent_dir + '/' + file_path
        self.file = open(self.file_path, "w")
        self.file.write("x,y,z,dim_x,dim_y,dim_z\n")
        self.total_poses = total_poses;

    def __del__(self):
        self.file.close()
        print "generated", self.total_poses, "datapoints in", self.file_path

    def generate(self):
        num_of_pose = 0
        while num_of_pose < self.total_poses:
            x = random.uniform(0.25, 0.7)
            y = random.uniform(0.0, 0.58)
            z = random.uniform(0.0, 0.2)

            self.file.write("{},{},{},{},{},{}\n".format(x, y, z, 0.06, 0.06, 0.06))
            num_of_pose += 1