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

#ifndef PROJECT_OBSTACLE_ANCHORING_TEST_H
#define PROJECT_OBSTACLE_ANCHORING_TEST_H

#include <jaco_manipulation/client/jaco_manipulation_client.h>
#include <jaco_manipulation/BoundingBox.h>
#include <anchor_msgs/AnchorArray.h>
#include <jaco_manipulation/units.h>

namespace jaco_manipulation {
namespace test {

struct Seconds {
  Seconds(size_t duration) : duration(duration) {}
  ~Seconds() = default;
  size_t duration;
};

using SeparatedObstacles = std::tuple<std::vector<jaco_manipulation::BoundingBox>, jaco_manipulation::BoundingBox, bool>;

class ObstacleAnchorTest {
 public:
  ObstacleAnchorTest() = delete;

  explicit ObstacleAnchorTest(const std::vector<BoundingBox> &datapoints);

  ~ObstacleAnchorTest() = default;

  void anchorArrayCallback(const anchor_msgs::AnchorArray::ConstPtr &msg);

  bool anchors_published() const;

 private:
  const std::vector<BoundingBox> &data;
  size_t trial_counter;
  size_t grip_counter;
  bool found_anchor;
  const std::string topic;
  bool time_to_add_obstacles_;
  double surface_coverage;
  int surface_obstacles_necessary;
  const int grasps_per_obstacle_coverage;
  BoundingBox drop_box;
  ros::NodeHandle n;
  ros::Subscriber sub;
  anchor_msgs::AnchorArray anchors;
  client::JacoManipulationClient jmc;
  std::vector<BoundingBox>::const_iterator current_box_it;

  std::vector<jaco_manipulation::BoundingBox>::const_iterator next_point();

  void show_summary(const std::vector<std::string> &labels) const;

  jaco_manipulation::BoundingBox createBoundingBoxFromAnchor(const anchor_msgs::Anchor &anchor, bool const_label = true) const;

  void countdown(struct Seconds seconds, bool target_found = true) const;
  SeparatedObstacles extractObstacles(const anchor_msgs::AnchorArray::ConstPtr &msg) const;

  void show_test_info();
};
} // namespace tes
} // namespace jaco_manipulation

#endif //PROJECT_OBSTACLE_ANCHORING_TEST_H
