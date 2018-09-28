#!/usr/bin/env python
# kinetic only support python 2.7

import rospy
import os
from jaco_manipulation.msg import JacoDebug


class Test:
    def __init__(self, filename):
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.filename = self.dir_path + '/' + filename
        self.__setup_log_file()

    def __setup_log_file(self):
        self.file = open(self.filename, "w")
        self.write("Time,CurrentPose,TargetPose,Result")

    def write(self, line):
        self.file.write(line + '\n')

    def pose_to_string(self, pose):
        return "({x},{y},{z})/({xx},{yy},{zz},{ww})".format(
            x=pose.pose.position.x, y=pose.pose.position.y, z=pose.pose.position.z,
            xx=pose.pose.orientation.x, yy=pose.pose.orientation.y,
            zz=pose.pose.orientation.z, ww=pose.pose.orientation.w
        )

    def __del__(self):
        self.file.close()
        print "Logged to", self.filename


class BaseLineTest(Test):
    def __init__(self, filename, delimiter=','):
        Test.__init__(self, filename)
        self.delimiter = delimiter
        rospy.init_node('base_line_tester', anonymous=True)
        rospy.Subscriber("jaco_manipulation/debug", JacoDebug, self.__callback)
        rospy.spin()

    def __msg_to_string(self, msg):
        time = msg.timestamp
        current_pose = Test.pose_to_string(self, msg.goal.current_pose)
        target_pose  = Test.pose_to_string(self, msg.goal.target_pose)
        result = 'success' if msg.result == 'True' else 'failure'
        return time + self.delimiter + current_pose + self.delimiter + target_pose + self.delimiter + result

    def __callback(self, msg):
        line = self.__msg_to_string(msg)
        self.__write(line)

    def __write(self, line):
        Test.write(self, line)


test = BaseLineTest("baseline.csv")
