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

#include <jaco_manipulation/test/anchoring_base_test.h>

using namespace jaco_manipulation::test;

AnchoringBaseTest::AnchoringBaseTest(const std::vector<jaco_manipulation::BoundingBox> &datapoints)
: data(datapoints),
  trial_counter(0),
  grip_counter(0),
  current_box_it(begin(data)),
  topic("/anchors")
{}

std::vector<jaco_manipulation::BoundingBox>::const_iterator AnchoringBaseTest::next_point() {
  if (current_box_it == end(data)-1) {
    return end(data);
  } else {
    return ++current_box_it;
  }
}

void AnchoringBaseTest::show_summary(const std::vector<std::string> &labels) const {
  const auto &target_label = labels[0];
  ROS_WARN_STREAM("-----");
  ROS_WARN_STREAM("Anchor " << target_label);
  std::stringstream ss;
  std::copy(begin(labels), end(labels), std::ostream_iterator<std::string>(ss," "));
  ROS_WARN_STREAM("Labels: " << ss.str());
  ROS_INFO_STREAM("Picking up anchor " << target_label);
  ROS_WARN_STREAM("-----");
}

void AnchoringBaseTest::show_test_info() {
  ROS_SUCCESS("----");
  ROS_SUCCESS("Test " << ++grip_counter);
  ROS_SUCCESS("Attempting to move to anchor");
  ROS_SUCCESS("----");
}
