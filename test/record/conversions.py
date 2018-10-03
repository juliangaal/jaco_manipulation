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
from geometry_msgs.msg import PoseStamped, Vector3, Point

class Conversions:
    @staticmethod
    def coords_to_point(x, y, z=0):
        msg = Point()
        msg.x = x;
        msg.y = y;
        msg.z = z;
        return msg

    @staticmethod
    def coords_to_pose(x, y, z=0):
        msg = PoseStamped()
        msg.header = rospy.Time.now()
        msg.pose.position = Conversions.coords_to_point(x, y, z)
        msg.pose.orientation.w = 1.
        return msg

    @staticmethod
    def shape_to_vec3(self, x=0.06, y=0.06, z=0.06):
        shape = Vector3()
        shape.x = x;
        shape.y = y;
        shape.z = z;
        return shape

    @staticmethod
    def pose_to_string(pose):
        return "({x},{y},{z})/({xx},{yy},{zz},{ww})".format(
            x=pose.pose.position.x, y=pose.pose.position.y, z=pose.pose.position.z,
            xx=pose.pose.orientation.x, yy=pose.pose.orientation.y,
            zz=pose.pose.orientation.z, ww=pose.pose.orientation.w
        )
