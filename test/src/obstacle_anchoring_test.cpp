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

#include <jaco_manipulation/test/baseline_csv_reader.h>
#include <jaco_manipulation/test/obstacle_anchoring_test.h>
#include <jaco_manipulation/units.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <sstream>
#include <chrono>
#include <thread>

using namespace jaco_manipulation::test;

ObstacleAnchorTest::ObstacleAnchorTest(const std::vector<BoundingBox> &datapoints)
: data(datapoints),
  trial_counter(0),
  grip_counter(0),
  current_box_it(begin(data)),
  found_anchor(false),
  topic("/anchors"),
  time_to_add_obstacles_(false),
  surface_coverage(11._percent),
  surface_obstacles_necessary(2),
  grasps_per_obstacle_coverage(10)
{
  drop_box.header.frame_id = "base_link";
  drop_box.description = "box";
  drop_box.point.x = 0.5_m;
  drop_box.point.y = 0.3_m;
  drop_box.point.z = 6.5_cm/2.;
  drop_box.dimensions.x = 6.5_cm;
  drop_box.dimensions.y = 6.5_cm;
  drop_box.dimensions.z = 6.5_cm;

  sub = n.subscribe(topic, 1, &ObstacleAnchorTest::anchorArrayCallback, this);
  sleep(1); // let anchoring system get up to speed

  if (data.empty()) {
    ROS_ERROR_STREAM("No data received! Did you generate poses?");
    ros::shutdown();
  }
}

void ObstacleAnchorTest::anchorArrayCallback(const anchor_msgs::AnchorArray::ConstPtr &msg) {
  static bool about_to_drop = false;

  if (msg->anchors.size() == 0) {
    ROS_WARN_STREAM("No anchors found. Skipping. . .");
    return;
  }

  static BoundingBox target_object;
  time_to_add_obstacles_ = false;

  if (trial_counter++ % 2 == 0) {

    std::vector<BoundingBox> obstacles;
    bool target_found;
    std::tie(obstacles, target_object, target_found) = this->extractObstacles(msg);

    if (not target_found) {
      --trial_counter;
      ROS_ERROR_STREAM("Target object not found. Skipping");
      countdown(Seconds(7), target_found);
      return;
    }

    if (obstacles.size() != surface_obstacles_necessary) {
      --trial_counter;
      ROS_WARN_STREAM("Incorrect number of obstacles (vs " << surface_obstacles_necessary << "). Returning");
      return;
    }

    std::vector<int> indices(obstacles.size());
    std::iota(begin(indices), end(indices), 1);
    for (auto it = begin(obstacles); it != end(obstacles); ++it)
    {
      auto box = *it;
      box.description = "obstacle-" + std::to_string(indices[std::distance(it, end(obstacles))-1]);
      jmc.updatePlanningScene(box);
    }

    show_test_info();

    jmc.graspAt(target_object);
    about_to_drop = true; // sometimes the anchoring system detectes the arm as  multiple obstacles
// set to true so the number of anchors are ignored after lifiting to get into callback to generate drop motion
  } else {
    jmc.dropAt(*current_box_it);

    if (next_point() == end(data)) {
      ROS_WARN_STREAM("Reached end of test.");
      ROS_WARN_STREAM("Waiting for last status from Jaco . . .");
      sleep(3);
      ROS_WARN_STREAM("Finishing up");
      sub.shutdown();
      ros::shutdown();
      return;
    }
  }
}

SeparatedObstacles ObstacleAnchorTest::extractObstacles(const anchor_msgs::AnchorArray::ConstPtr &msg) const {
  const auto &anchors = msg->anchors;

  auto contains = [](std::string str, std::string substr) { return str.find(substr) != std::string::npos; };
  auto target_anchor_it = std::find_if(begin(anchors), end(anchors), [&](const auto &anchor) {
    return contains(anchor.x, "ball");
  });

  auto target_box = jaco_manipulation::BoundingBox();
  bool target_found = target_anchor_it != end(anchors);
  if (target_found)
  {
    target_box = createBoundingBoxFromAnchor(*target_anchor_it);
  }
  else
  {
    target_box = createBoundingBoxFromAnchor(*begin(anchors));
  }

  std::vector<anchor_msgs::Anchor> obstacle_anchors;
  std::copy_if(begin(anchors), end(anchors), std::back_inserter(obstacle_anchors), [&](const auto& anchor) {
    return !contains(anchor.x, "ball");
  });

  std::vector<BoundingBox> obstacles;
  std::transform(begin(obstacle_anchors), end(obstacle_anchors), std::back_inserter(obstacles), [this](const auto &anchor) {
    return this->createBoundingBoxFromAnchor(anchor, false);
  });

  for (const auto& anchor: anchors) {
    show_summary(anchor.caffe.symbols);
  }

  ROS_WARN_STREAM("Will add " << obstacle_anchors.size() << " obstacles to planning scene");

  return std::make_tuple(obstacles, target_box, target_found);
}

jaco_manipulation::BoundingBox ObstacleAnchorTest::createBoundingBoxFromAnchor(const anchor_msgs::Anchor &anchor, bool const_label) const {
  jaco_manipulation::BoundingBox box;
  box.header.frame_id = "base_link";
//  box.header.stamp = ros::Time::now();
  if (const_label) {
    // target label has to be the same for all boxes in the test. This way the old target gets replaced
    // with the new target, not added! The how MoveIt handles objects in moveit_visuals
    box.description = "box";
  } else {
    box.description = anchor.x;
  }
  box.point = anchor.position.data.pose.position;
  box.point.x += anchor.shape.data.x * 0.5; // correction: centroid is infront of bounding box from kinect
  box.dimensions = anchor.shape.data;

  return box;
}

bool ObstacleAnchorTest::anchors_published() const {
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  auto topic_found_it = std::find_if(begin(master_topics), end(master_topics), [&](auto &top) {
    return top.name == topic;
  });

  return topic_found_it != end(master_topics);
}

std::vector<jaco_manipulation::BoundingBox>::const_iterator ObstacleAnchorTest::next_point() {
  if (current_box_it == end(data)-1) {
    return end(data);
  } else {
    return ++current_box_it;
  }
}

void ObstacleAnchorTest::show_summary(const std::vector<std::string> &labels) const {
  const auto &target_label = labels[0];
  ROS_WARN_STREAM("-----");
  ROS_WARN_STREAM("Anchor " << target_label);
  std::stringstream ss;
  std::copy(begin(labels), end(labels), std::ostream_iterator<std::string>(ss," "));
  ROS_WARN_STREAM("Labels: " << ss.str());
  ROS_INFO_STREAM("Picking up anchor " << target_label);
  ROS_WARN_STREAM("-----");
}

void ObstacleAnchorTest::countdown(struct Seconds seconds, bool target_found) const
{
  using namespace std;
  chrono::seconds countdown(seconds.duration);
  chrono::seconds stop(1);

  while (countdown.count() > 0)
  {
    std::this_thread::sleep_for(stop);
    countdown -= stop;
    ROS_WARN("---------");
    ROS_WARN_STREAM("Continuing in " << countdown.count() << " seconds");
    ROS_WARN("---------\n");

    if (not target_found)
      ROS_ERROR_STREAM("Rearrange target!");

    if (time_to_add_obstacles_)
      ROS_ERROR_STREAM("ADD NEW OBSTACLES!");
  }
}

void ObstacleAnchorTest::show_test_info() {
  ROS_SUCCESS("----");
  ROS_SUCCESS("Test " << ++grip_counter);
  ROS_SUCCESS("Attempting to move to anchor");
  ROS_SUCCESS("----");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ObstacleObstacleAnchorTest");

  BaselineCSVReader reader("/home/chitt/julian/reground_workspace/src/arm/jaco_manipulation/scripts/anchoring_poses.csv");
  auto data = reader.getData();
  ObstacleAnchorTest l(data);

  if (!l.anchors_published()) {
    ROS_WARN_STREAM("Anchors don't appear to be published");
  }

  ROS_INFO_STREAM("Starting anchoring test");

  while(!ros::isShuttingDown()) {
    ros::spinOnce();
  }

  return 0;
}
