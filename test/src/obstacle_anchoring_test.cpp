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
#include <algorithm>

using namespace jaco_manipulation::test;
using std::vector;
using std::tuple;
using jaco_manipulation::BoundingBox;

ObstacleAnchorTest::ObstacleAnchorTest(const vector<BoundingBox> &data)
: AnchorBaseTest(data),
  found_anchor_(false)
{
  sub_ = nh_.subscribe(topic_, 1, &ObstacleAnchorTest::anchorArrayCallback, this);
  sleep(1); // let anchoring system get up to speed

  if (data.empty()) {
    ROS_ERROR_STREAM("No data received! Did you generate poses?");
    ros::shutdown();
  }
}

void ObstacleAnchorTest::anchorArrayCallback(const anchor_msgs::AnchorArray::ConstPtr &msg) {
  if (msg->anchors.size() == 0) {
    ROS_WARN_STREAM("No anchors found. Skipping. . .");
    return;
  }

  if (trial_counter_++ % 2 == 0) {
    AnchorBaseTest::show_test_info();


    vector<BoundingBox> obstacles;
    BoundingBox target_object;
    std::tie(obstacles, target_object) = extractObstacles(msg);
//    for (const auto &obstacle: obstacles) {
////      jmc_.addObstacle(obstacle);
//    }

//    jmc_.graspAt(target_object);
  } else {
//    jmc_.dropAt(adaptDropBoxToAnchorDims(current_drop_box_it_));

    if (next_drop_box() == end(data_)) {
      ROS_WARN_STREAM("Reached end of test.");
      ROS_WARN_STREAM("Waiting for last status from Jaco . . .");
      sleep(3);
      ROS_WARN_STREAM("Finishing up");
      sub_.shutdown();
      ros::shutdown();
      return;
    }
  }
}

bool ObstacleAnchorTest::anchors_published() const {
  return false;
}

SeparatedObstacles ObstacleAnchorTest::extractObstacles(const anchor_msgs::AnchorArray::ConstPtr &msg) const {
  const auto &anchors = msg->anchors;

  auto contains = [](std::string str, std::string substr) { return str.find(substr) != std::string::npos; };
  auto target_anchor_it = std::find_if(begin(anchors), end(anchors), [&](const auto &anchor) {
    return contains(anchor.x, "ball");
  });
  auto target_box = AnchorBaseTest::createBoundingBoxFromAnchor(*target_anchor_it);

  vector<anchor_msgs::Anchor> obstacle_anchors;
  std::copy_if(begin(anchors), end(anchors), std::back_inserter(obstacle_anchors), [&](const auto& anchor) {
    return !contains(anchor.x, "ball");
  });

  vector<BoundingBox> obstacles;
////  std::transform(begin(obstacle_anchors), end(obstacle_anchors), begin(obstacles), [this](const auto &anchor) {
////    return AnchorBaseTest::createBoundingBoxFromAnchor(anchor);
////  }); WHY NO WORKY
  for (const auto& anchor: obstacle_anchors) {
    obstacles.push_back(AnchorBaseTest::createBoundingBoxFromAnchor(anchor));
  }

  for (const auto& anchor: anchors) {
    AnchorBaseTest::show_summary(anchor.caffe.symbols);
  }

  ROS_WARN_STREAM("Will add " << obstacle_anchors.size() << " to planning scene");

  return std::make_tuple(obstacles,target_box);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ObstacleAnchorTest");

  BaselineCSVReader reader("/home/chitt/julian/reground_workspace/src/arm/jaco_manipulation/scripts/obstacle_anchoring_poses.csv");
  const auto &data = reader.getData();
  ObstacleAnchorTest l(data);

  if (!l.anchors_published()) {
    ROS_WARN_STREAM("Anchors don't appear to be published");
  }

  ROS_INFO_STREAM("Starting obstacle anchoring test");

  while (!ros::isShuttingDown()) {
    ros::spinOnce();
  }

  return 0;
}
