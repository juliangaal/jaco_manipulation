#!/usr/bin/env python
import os
import random


class PoseGenerator:
    def __init__(self, file_path, total_poses):
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.file_path = self.dir_path + '/' + file_path
        self.file = open(self.file_path, "w")
        self.file.write("x,y,z,dim_x,dim_y,dim_z\n")
        self.total_poses = total_poses;
        self.__generate()

    def __del__(self):
        self.file.close()
        print "generated", self.total_poses, "datapoints in", self.file_path

    def __generate(self):
        num_of_pose = 0
        while (num_of_pose < self.total_poses):
            x = random.uniform(0.25, 0.7)
            y = random.uniform(0.0, 0.58)
            z = random.uniform(0.0, 0.2)

            self.file.write("{},{},{},{},{},{}\n".format(x, y, z, 0.06, 0.06, 0.06))
            num_of_pose += 1