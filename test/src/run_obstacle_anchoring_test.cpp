/*
  Copyright (C) 2018  Julian Gaal
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include <jaco_manipulation/test/obstacle_anchoring_test.h>
#include <jaco_manipulation/test/baseline_csv_reader.h>

using namespace jaco_manipulation::test;

int main(int argc, char **argv) {
  ros::init(argc, argv, "ObstacleAnchorTest");

  BaselineCSVReader reader("/home/chitt/julian/reground_workspace/src/arm/jaco_manipulation/scripts/obstacle_anchoring_poses.csv");
  const auto &data = reader.getData();
  ObstacleAnchorTest l(data);

  if (!l.anchors_published()) {
    ROS_ERROR_STREAM("Anchors don't appear to be published");
  }

  ROS_INFO_STREAM("Starting obstacle anchoring test");

  while (!ros::isShuttingDown()) {
    ros::spinOnce();
  }

  return 0;
}