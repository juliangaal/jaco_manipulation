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

#include <jaco_manipulation/goals/vision_goal.h>
#include <ros/console.h>
#include <tf/tf.h>

using namespace jaco_manipulation::goals;

VisionGoal::VisionGoal(const goal_input::LimitedPose &grasp_pose_goal,
                       jaco_manipulation::grasps::GraspType grasp,
                       const std::string &description)
: requested_grasp_(grasp) {
  description_ = description;

  goal_.pose_goal.header.frame_id = planning_frame_;
  goal_.goal_type = "goal";

  goal_.pose_goal.pose.position.x = grasp_pose_goal.x;
  goal_.pose_goal.pose.position.y = grasp_pose_goal.y;
  goal_.pose_goal.pose.position.z = grasp_pose_goal.z;

//  adjustOrientation(grasp);
}

VisionGoal::VisionGoal(const jaco_manipulation::BoundingBox &bounding_box_goal,
                       jaco_manipulation::grasps::GraspType grasp,
                       const std::string &description)
: requested_grasp_(grasp) {
  description_ = description + " \"" + bounding_box_goal.description + "\"";

  goal_.pose_goal.header.frame_id = planning_frame_;
  goal_.goal_type = "goal";
  goal_.bounding_box = bounding_box_goal;

  adjustPose(grasp, bounding_box_goal);
}

jaco_manipulation::PlanAndMoveArmGoal VisionGoal::goal() const {
  return PoseGoal::goal();
}

void VisionGoal::adjustPose(jaco_manipulation::grasps::GraspType grasp,
                            const jaco_manipulation::BoundingBox &box) {
  grasp_orientation_generator_.adjustPose(goal_.pose_goal, box, grasp);
}

std::string VisionGoal::requestedOrientation() {
  using jaco_manipulation::grasps::GraspType;
  switch (requested_grasp_) {
    case GraspType::TOP_GRASP:
      return "Top Orientation";
    case GraspType::FRONT_GRASP:
      return "Frontal Orientation";
    case GraspType::LEFT_GRASP:
      return "Left Orientation";
    case GraspType::RIGHT_GRASP:
      return "Right Orientation";
    default:
      return "Any Orientation";
  }
}
