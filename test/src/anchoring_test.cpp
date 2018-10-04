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
#include <jaco_manipulation/test/baseline_csv_reader.h>
#include <jaco_manipulation/test/anchoring_test.h>

using namespace jaco_manipulation::test;

AnchorTest::AnchorTest(const std::vector<BoundingBox> &datapoints)
    : AnchorBaseTest(datapoints),
      found_anchor_(false) 
{
  sub_ = nh_.subscribe(topic_, 1, &AnchorTest::anchorArrayCallback, this);
  sleep(1); // let anchoring system get up to speed

  if (data_.empty()) {
    ROS_ERROR_STREAM("No data received! Did you generate poses?");
    ros::shutdown();
  }
}

void AnchorTest::anchorArrayCallback(const anchor_msgs::AnchorArray::ConstPtr &msg) {
  static bool about_to_drop = false;
  if (msg->anchors.size() > 1 && !about_to_drop) {
    ROS_WARN_STREAM("Too many active anchors. Skipping. . .");
    return;
  }

  if (msg->anchors.size() == 0) {
    ROS_WARN_STREAM("No anchors found. Skipping. . .");
    return;
  }

  anchors_ = *msg;

  if (trial_counter_++ % 2 == 0) {
    AnchorBaseTest::show_test_info();
    current_anchor_box_ = createBoundingBoxFromAnchors();
    jmc_.graspAt(current_anchor_box_);
    about_to_drop = true; // sometimes the anchoring system detectes the arm as  multiple obstacles
    // set to true so the number of anchors are ignored after lifiting to get into callback to generate drop motion
  } else {
    jmc_.dropAt(adaptDropBoxToAnchorDims(current_drop_box_it_));
    about_to_drop = false;
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

bool AnchorTest::anchors_published() const {
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  auto topic_found_it = std::find_if(begin(master_topics), end(master_topics), [&](auto &top) {
    return top.name == topic_;
  });

  return topic_found_it != end(master_topics);
}

jaco_manipulation::BoundingBox AnchorTest::createBoundingBoxFromAnchors() const {
  if (anchors_.anchors.size() > 1) {
    ROS_WARN_STREAM("Can't create bounding box from anchor array > 1");
    return drop_box_;
  }

  const auto &anchor = anchors_.anchors[0];
  const auto &poss_labels = anchor.caffe.symbols;
  const auto &target_label = poss_labels[0];
  AnchorBaseTest::show_summary(poss_labels);

  return AnchorBaseTest::createBoundingBoxFromAnchor(anchor);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "AnchorTest");

  BaselineCSVReader reader("/home/chitt/julian/reground_workspace/src/arm/jaco_manipulation/scripts/anchoring_poses.csv");
  const auto &data = reader.getData();
  AnchorTest l(data);

  if (!l.anchors_published()) {
    ROS_WARN_STREAM("Anchors don't appear to be published");
  }

  ROS_INFO_STREAM("Starting anchoring test");

  while (!ros::isShuttingDown()) {
    ros::spinOnce();
  }

  return 0;
}
