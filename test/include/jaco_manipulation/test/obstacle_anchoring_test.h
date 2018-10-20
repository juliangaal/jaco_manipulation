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

#include <jaco_manipulation/test/anchoring_base_test.h>
#include <tuple>

namespace jaco_manipulation {
namespace test {

struct Seconds {
  Seconds(size_t duration) : duration(duration) {}
  ~Seconds() = default;
  size_t duration;
};

using SeparatedObstacles = std::tuple<std::vector<jaco_manipulation::BoundingBox>, jaco_manipulation::BoundingBox, bool>;

class ObstacleAnchorTest : public AnchorBaseTest {
 public:
  ObstacleAnchorTest() = delete;
  explicit ObstacleAnchorTest(const std::vector<jaco_manipulation::BoundingBox> &data);
  ~ObstacleAnchorTest() final = default;

  void anchorArrayCallback(const anchor_msgs::AnchorArray::ConstPtr &msg);

  bool anchors_published() const;

 private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  bool found_anchor_;
  bool time_to_add_obstacles_;
  double surface_coverage;
  int surface_obstacles_necessary;
  const int grasps_per_obstacle_coverage;
  void countdown(struct Seconds seconds, bool target_found = true) const;
  SeparatedObstacles extractObstacles(const anchor_msgs::AnchorArray::ConstPtr &msg) const;
};
} // namespace test
} // namespace jaco_manipulation

#endif //PROJECT_OBSTACLE_ANCHORING_TEST_H
