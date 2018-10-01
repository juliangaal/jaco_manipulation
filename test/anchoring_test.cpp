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

#include <jaco_manipulation/client/jaco_manipulation_client.h>
#include <jaco_manipulation/test/baseline_csv_reader.h>
#include <anchor_msgs/AnchorArray.h>
#include <jaco_manipulation/BoundingBox.h>
#include <jaco_manipulation/units.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <sstream>

using namespace jaco_manipulation;

class AnchorTest {
 public:
  AnchorTest(const std::vector<BoundingBox> &datapoints, std::string anchor="ball")
  : data(datapoints),
    trial_counter(0),
    current_box_it(begin(data)),
    found_anchor(false),
    target_anchor(anchor),
    topic("/anchors")
  {
    drop_box.header.frame_id = "base_link";
    drop_box.description = "box";
    drop_box.point.x = 0.5_m;
    drop_box.point.y = 0.3_m;
    drop_box.point.z = 6.5_cm/2.;
    drop_box.dimensions.x = 6.5_cm;
    drop_box.dimensions.y = 6.5_cm;
    drop_box.dimensions.z = 6.5_cm;

    sub = n.subscribe(topic, 1, &AnchorTest::anchorArrayCallback, this);
    sleep(1); // let anchoring system get up to speed
  }

  ~AnchorTest() = default;

  void anchorArrayCallback(const anchor_msgs::AnchorArray::ConstPtr &msg) {
    anchors = *msg;

    ROS_INFO_STREAM("Attempting to find anchor");

    if (trial_counter % 2 == 0) {
      auto box = createBoundingBoxFromAnchors(anchors);
      if (found_anchor) {
        jmc.graspAt(box);
      } else {
        ROS_WARN_STREAM("No anchor 'ball' found. Skipping...");
        return;
      }
    } else {
      jmc.dropAt(*current_box_it);
    }

    sleep(3); // sleep so anchoring has time to correct itself
    ++trial_counter;

    if (next_point() == end(data)-1) {
      ROS_WARN_STREAM("Reached end of test.");
      ROS_WARN_STREAM("Waiting for last status from Jaco . . .");
      sleep(3);
      ROS_WARN_STREAM("Finishing up");
      sub.shutdown();
      ros::shutdown();
      return;
    }
  }

  bool anchors_published() const {
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    auto topic_found_it = std::find_if(begin(master_topics), end(master_topics), [&](auto &top) {
      return top.name == topic;
    });

    return topic_found_it != end(master_topics);
  }

 private:
    const std::vector<BoundingBox> &data;
    size_t trial_counter;
    bool found_anchor;
    const std::string target_anchor;
    const std::string topic;
    BoundingBox drop_box;
    ros::NodeHandle n;
    ros::Subscriber sub;
    anchor_msgs::AnchorArray anchors;
    client::JacoManipulationClient jmc;

    std::vector<BoundingBox>::const_iterator current_box_it;

    std::vector<BoundingBox>::const_iterator next_point() {
      if (current_box_it == end(data)) {
        return end(data);
      } else {
        return ++current_box_it;
      }
    }

    jaco_manipulation::BoundingBox createBoundingBoxFromAnchors(const anchor_msgs::AnchorArray anchors) {
      size_t anchor_num = 0;

      for (const auto& anchor: anchors.anchors) {
        auto &poss_labels = anchor.caffe.symbols;

        show_summary(poss_labels, ++anchor_num, anchors.anchors.size());

        auto anchor_it = std::find_if(begin(poss_labels), end(poss_labels), [&](auto label) {
          return label == target_anchor;
        });

        if (anchor_it != end(poss_labels) && poss_labels[0] == "ball") {
          found_anchor = true;

          jaco_manipulation::BoundingBox box;
          box.header.frame_id = "base_link";
          box.description = target_anchor;
          box.point = anchor.position.data.pose.position;
          box.point.x += anchor.shape.data.x * 0.5; // correction: centroid is infront of bounding box from kinect
          box.dimensions = anchor.shape.data;
          return box;
        }
      }

      ROS_ERROR_STREAM("Can't create bounding box from current anchors. Defaulting to drop box");
      found_anchor = false;
      return drop_box;
    }

    void show_summary(const std::vector<std::string> &labels, size_t current_anchor, size_t num_of_anchors) const {
      ROS_WARN_STREAM("-----");
      ROS_WARN_STREAM("Anchor " << current_anchor << "/" << num_of_anchors);
      std::stringstream ss;
      std::copy(begin(labels), end(labels),std::ostream_iterator<std::string>(ss," "));
      ROS_WARN_STREAM("Labels: " << ss.str());
      ROS_WARN_STREAM("-----");
      auto target_it = std::find_if(begin(labels), end(labels), [&](auto &label) { return label == target_anchor; });
      if (target_it != end(labels))
        ROS_SUCCESS("Found target anchor. Moving!");
    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "AnchorTest");

  test::BaselineCSVReader reader("/home/chitt/julian/reground_workspace/src/arm/jaco_manipulation/scripts/anchoring_poses.csv");
  auto data = reader.getData();
  AnchorTest l(data);

  ros::Rate loop_rate(1);
  while(!l.anchors_published()) {
    ROS_WARN_STREAM("Anchors don't appear to be published");
    loop_rate.sleep();
  }

  ROS_INFO_STREAM("Starting anchoring test");

  while(!ros::isShuttingDown()) {
    ros::spinOnce();
  }

  return 0;
}
