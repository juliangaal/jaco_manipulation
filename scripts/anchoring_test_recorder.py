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
from test import Test


class AnchoringTest(Test):
    def __init__(self, filename, labels='anchoring labels', delimiter=','):
        Test.__init__(self, filename, labels)
        self.delimiter = delimiter
        rospy.init_node('anchoring_tester', anonymous=True)
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rospy.Subscriber("jaco_manipulation/debug", JacoDebug, self.__callback)
        rospy.spin()

    def __callback(self, msg):
        line = self.__msg_to_string(msg)
        self.__write(line)

    def __msg_to_string(self, msg):
        time = msg.timestamp.strip('\n')
        command = '"Pick up the ball"'
        current_pose = Test.pose_to_string(self, msg.goal.current_pose).strip('\n')
        target_pose  = Test.pose_to_string(self, msg.goal.target_pose).strip('\n')
        result = ('success' if True else 'failure').strip('\n')
        return time + self.delimiter + command + self.delimiter + current_pose + \
               self.delimiter + target_pose + self.delimiter + result

    def __write(self, line):
        Test.write(self, line)


anchor_test = AnchoringTest("anchoring_test_recording.csv", "Time,Command,CurrentPose,TargetPose,Result")