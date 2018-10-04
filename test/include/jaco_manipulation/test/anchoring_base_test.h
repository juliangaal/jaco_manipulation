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

#ifndef PROJECT_ANCHORING_BASE_TEST_H
#define PROJECT_ANCHORING_BASE_TEST_H

#include <jaco_manipulation/client/jaco_manipulation_client.h>
#include <jaco_manipulation/BoundingBox.h>
#include <anchor_msgs/AnchorArray.h>

namespace jaco_manipulation {
namespace test {

class AnchoringBaseTest {
 public:
  virtual ~AnchoringBaseTest() = default;

 protected:
  AnchoringBaseTest() = default;
  explicit AnchoringBaseTest(const std::vector<BoundingBox> &datapoints);
  const std::vector<jaco_manipulation::BoundingBox> &data;
  size_t trial_counter;
  size_t grip_counter;
  const std::string topic;
  anchor_msgs::AnchorArray anchors;
  client::JacoManipulationClient jmc;
  std::vector<BoundingBox>::const_iterator current_box_it;

  std::vector<jaco_manipulation::BoundingBox>::const_iterator next_point();
  void show_summary(const std::vector<std::string> &labels) const;
  void show_test_info();
};
} // namespace test
} // namespace jaco_manipulation

#endif //PROJECT_ANCHORING_BASE_TEST_H
