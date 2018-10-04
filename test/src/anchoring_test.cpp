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
      found_anchor(false) {
  drop_box.header.frame_id = "base_link";
  drop_box.description = "box";
  drop_box.point.x = 0.5_m;
  drop_box.point.y = 0.3_m;
  drop_box.point.z = 6.5_cm / 2.;
  drop_box.dimensions.x = 6.5_cm;
  drop_box.dimensions.y = 6.5_cm;
  drop_box.dimensions.z = 6.5_cm;

  sub = n.subscribe(topic, 1, &AnchorTest::anchorArrayCallback, this);
  sleep(1); // let anchoring system get up to speed

  if (data.empty()) {
    ROS_ERROR_STREAM("No data received! Did you generate poses?");
    ros::shutdown();
  }
}

void AnchorTest::anchorArrayCallback(const anchor_msgs::AnchorArray::ConstPtr &msg) {
  if (msg->anchors.size() > 1) {
    ROS_WARN_STREAM("Too many active anchors. Skipping. . .");
    return;
  }

  if (msg->anchors.size() == 0) {
    ROS_WARN_STREAM("No anchors found. Skipping. . .");
    return;
  }

  anchors = *msg;

  if (trial_counter++ % 2 == 0) {
    AnchorBaseTest::show_test_info();
    current_anchor_box = createBoundingBoxFromAnchors();
    jmc.graspAt(current_anchor_box);
  } else {
    jmc.dropAt(adaptDropBoxToAnchorDims(current_drop_box_it));

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

bool AnchorTest::anchors_published() const {
  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  auto topic_found_it = std::find_if(begin(master_topics), end(master_topics), [&](auto &top) {
    return top.name == topic;
  });

  return topic_found_it != end(master_topics);
}

jaco_manipulation::BoundingBox AnchorTest::createBoundingBoxFromAnchors() const {
  if (anchors.anchors.size() > 1) {
    ROS_WARN_STREAM("Can't create bounding box from anchor array > 1");
    return drop_box;
  }

  const auto &anchor = anchors.anchors[0];
  const auto &poss_labels = anchor.caffe.symbols;
  const auto &target_label = poss_labels[0];
  AnchorBaseTest::show_summary(poss_labels);

  jaco_manipulation::BoundingBox box;
  box.header.frame_id = "base_link";
  // target label has to be the same for all boxes in the test. This way the old target gets replaced
  // with the new target, not added! The how MoveIt handles objects in moveit_visuals
  box.description = "box";
  box.point = anchor.position.data.pose.position;
  box.point.x += anchor.shape.data.x * 0.5; // correction: centroid is infront of bounding box from kinect
  box.dimensions = anchor.shape.data;

  return box;
}

jaco_manipulation::BoundingBox
AnchorTest::adaptDropBoxToAnchorDims(std::vector<jaco_manipulation::BoundingBox>::const_iterator current_drop_box_it) const {
  jaco_manipulation::BoundingBox box = *current_drop_box_it;
  box.dimensions = current_anchor_box.dimensions;
  box.point.z = box.dimensions.z / 2.;
  return box;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "AnchorTest");

  BaselineCSVReader
      reader("/home/chitt/julian/reground_workspace/src/arm/jaco_manipulation/scripts/anchoring_poses.csv");
  auto data = reader.getData();
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
