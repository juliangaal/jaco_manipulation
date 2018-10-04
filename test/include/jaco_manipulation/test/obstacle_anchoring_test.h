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

namespace jaco_manipulation {
namespace test {

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
};
} // namespace test
} // namespace jaco_manipulation

#endif //PROJECT_OBSTACLE_ANCHORING_TEST_H
