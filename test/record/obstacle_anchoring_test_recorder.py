#!/usr/bin/env python
# Copyright (C) 2018  Julian Gaal
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#     You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>

import rospy
from jaco_manipulation.msg import JacoDebug
from std_msgs.msg import String
from conversions import Conversions
from record.test_recorder import Test

class AnchoringObstacleTest(Test):
    def __init__(self, filename, labels='anchoring labels', delimiter=';'):
        Test.__init__(self, filename, labels)
        self.delimiter = delimiter
        self.percent = 11
        self.run_number = 1
        rospy.init_node('anchoring_tester', anonymous=True)
        rospy.Subscriber("jaco_manipulation/debug", JacoDebug, self.__callback)
        rospy.spin()

    def __callback(self, msg):
        line = self.__msg_to_string(msg)
        self.__write(line)
        if msg.goal.description == "grasp_pose":
            if self.run_number % 11 == 0:
                self.percent += 11

            self.run_number += 1

    def __msg_to_string(self, msg):
        type_ = msg.goal.description
        time = msg.timestamp.strip('\n')
        current_pose = Conversions.pose_to_string(msg.goal.current_pose).strip('\n')
        target_pose  = Conversions.pose_to_string(msg.goal.target_pose).strip('\n')
        num_obstacles = str(msg.planning_scene.num_of_obstacles)
        result = msg.result.strip('\n')
        return time + self.delimiter + type_ + self.delimiter + current_pose + \
               self.delimiter + target_pose + self.delimiter + num_obstacles + self.delimiter + result  +\
               self.delimiter + "Default" + self.delimiter + str(self.percent)

    def __write(self, line):
        Test.write(self, line)


if __name__ == "__main__":
    anchor_test = AnchoringObstacleTest("obstacle_anchoring_test_recording.csv",
                                        "Time;Type;CurrentPose;TargetPose;Obstacles;Result;Gripped;Percent")
