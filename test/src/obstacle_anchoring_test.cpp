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

#include <jaco_manipulation/test/obstacle_anchoring_test.h>
#include <jaco_manipulation/test/baseline_csv_reader.h>
#include <jaco_manipulation/units.h>
#include <algorithm>
#include <chrono>
#include <thread>

using namespace jaco_manipulation::test;
using std::vector;
using std::tuple;
using jaco_manipulation::BoundingBox;

ObstacleAnchorTest::ObstacleAnchorTest(const vector<BoundingBox> &data)
: AnchorBaseTest(data),
  found_anchor_(false),
  time_to_add_obstacles_(false),
  surface_coverage(11._percent),
  surface_obstacles_necessary(2),
  grasps_per_obstacle_coverage(10)
{
  ROS_INFO_STREAM("Number of drop poses: " << data.size());
  sub_ = nh_.subscribe(topic_, 1, &ObstacleAnchorTest::anchorArrayCallback, this);
  sleep(1); // let anchoring system get up to speed

  if (data.empty()) {
    ROS_ERROR_STREAM("No data received! Did you generate poses?");
    ros::shutdown();
  }
}

void ObstacleAnchorTest::anchorArrayCallback(const anchor_msgs::AnchorArray::ConstPtr &msg) {
  if (msg->anchors.size() == 0) {
    ROS_WARN_STREAM("No anchors found. Skipping. . .");
    return;
  }
  static BoundingBox target_object;

  time_to_add_obstacles_ = false;

  if (trial_counter_++ % 2 == 0) {
    vector<BoundingBox> obstacles;
    bool target_found;
    std::tie(obstacles, target_object, target_found) = this->extractObstacles(msg);

    if (not target_found) {
      --trial_counter_;
      ROS_ERROR_STREAM("Target object not found. Skipping");
      countdown(Seconds(7), target_found);
      return;
    }

    if (obstacles.size() != surface_obstacles_necessary) {
      --trial_counter_;
      ROS_WARN_STREAM("Incorrect number of obstacles (vs " << surface_obstacles_necessary << "). Returning");
      return;
    }

    std::vector<int> indices(obstacles.size());
    std::iota(begin(indices), end(indices), 1);
    for (auto it = begin(obstacles); it != end(obstacles); ++it)
    {
      auto box = *it;
      box.description = "obstacle-" + std::to_string(indices[std::distance(it, end(obstacles))-1]);
      jmc_.updatePlanningScene(box);
    }

    AnchorBaseTest::show_test_info(target_object.description);

    jmc_.graspAt(target_object);
  } else {
    jmc_.dropAt(target_object);

    if (next_drop_box() == end(data_) or trial_counter_ == 30) {
      ROS_WARN_STREAM("Reached end of test.");
      ROS_WARN_STREAM("Waiting for last status from Jaco . . .");
      sleep(3);
      ROS_WARN_STREAM("Finishing up");
      sub_.shutdown();
      ros::shutdown();
      return;
    }
    jmc_.wipePlanningScene();
    if (grip_counter_ % grasps_per_obstacle_coverage == 0 and not grip_counter_ == 0) {
      time_to_add_obstacles_ = true;
      countdown(Seconds(10));
      surface_coverage += 11._percent;
      surface_obstacles_necessary += 2;
      return;
    }
    countdown(Seconds(10));
  }
}

bool ObstacleAnchorTest::anchors_published() const {
  return false;
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
    target_box = AnchorBaseTest::createBoundingBoxFromAnchor(*target_anchor_it);
  }
  else
  {
    target_box = AnchorBaseTest::createBoundingBoxFromAnchor(*begin(anchors));
  }

  vector<anchor_msgs::Anchor> obstacle_anchors;
  std::copy_if(begin(anchors), end(anchors), std::back_inserter(obstacle_anchors), [&](const auto& anchor) {
    return !contains(anchor.x, "ball");
  });

  vector<BoundingBox> obstacles;
  std::transform(begin(obstacle_anchors), end(obstacle_anchors), std::back_inserter(obstacles), [this](const auto &anchor) {
    return this->createBoundingBoxFromAnchor(anchor, false);
  });

  for (const auto& anchor: anchors) {
    AnchorBaseTest::show_summary(anchor.caffe.symbols);
  }

  ROS_WARN_STREAM("Will add " << obstacle_anchors.size() << " obstacles to planning scene");

  return std::make_tuple(obstacles, target_box, target_found);
}

void ObstacleAnchorTest::countdown(struct Seconds seconds, bool target_found) const
{
  using namespace std;
  chrono::seconds countdown(seconds.duration);
  chrono::seconds stop(1);

  while (countdown.count() > 0)
  {
    this_thread::sleep_for(stop);
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

int main(int argc, char **argv) {
  ros::init(argc, argv, "ObstacleAnchorTest");

  BaselineCSVReader reader("/home/chitt/julian/reground_workspace/src/arm/jaco_manipulation/scripts/obstacle_anchoring_poses.csv");
  const auto &data = reader.getData();
  ObstacleAnchorTest l(data);

  if (!l.anchors_published()) {
    ROS_ERROR_STREAM("Anchors don't appear to be published");
  }

  ROS_INFO_STREAM("Starting obstacle anchoring test");

  while (!ros::isShuttingDown()) {
    ros::spinOnce();
  }

  return 0;
}
