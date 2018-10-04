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

AnchorBaseTest::AnchorBaseTest(const std::vector<jaco_manipulation::BoundingBox> &datapoints)
: data_(datapoints),
  trial_counter_(0),
  grip_counter_(0),
  current_drop_box_it_(begin(data_)),
  topic_("/anchors")
{
  drop_box_.header.frame_id = "base_link";
  drop_box_.description = "box";
  drop_box_.point.x = 0.5_m;
  drop_box_.point.y = 0.3_m;
  drop_box_.point.z = 6.5_cm / 2.;
  drop_box_.dimensions.x = 6.5_cm;
  drop_box_.dimensions.y = 6.5_cm;
  drop_box_.dimensions.z = 6.5_cm;
}

std::vector<jaco_manipulation::BoundingBox>::const_iterator AnchorBaseTest::next_point() {
  if (current_drop_box_it_ == end(data_)) {
    ROS_ERROR_STREAM("Reached end of data. But node did not successfully call ros::shudown(). This node will crash");
    return end(data_);
  }

  if (current_drop_box_it_ == end(data_)-1) {
    return end(data_);
  } else {
    return ++current_drop_box_it_;
  }
}

void AnchorBaseTest::show_summary(const std::vector<std::string> &labels) const {
  const auto &target_label = labels[0];
  ROS_WARN_STREAM("-----");
  ROS_WARN_STREAM("Anchor " << target_label);
  std::stringstream ss;
  std::copy(begin(labels), end(labels), std::ostream_iterator<std::string>(ss," "));
  ROS_WARN_STREAM("Labels: " << ss.str());
  ROS_INFO_STREAM("Picking up anchor " << target_label);
  ROS_WARN_STREAM("-----");
}

void AnchorBaseTest::show_test_info() {
  ROS_SUCCESS("----");
  ROS_SUCCESS("Test " << ++grip_counter_);
  ROS_SUCCESS("Attempting to move to anchor");
  ROS_SUCCESS("----");
}
