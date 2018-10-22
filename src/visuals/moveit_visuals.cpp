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

#include <jaco_manipulation/visuals/moveit_visuals.h>
#include <jaco_manipulation/server/jaco_manipulation_server.h>
#include <jaco_manipulation/units.h>

using namespace jaco_manipulation::visuals;

MoveitVisuals::MoveitVisuals(ros::NodeHandle &nh, const std::string frame,
                             moveit::planning_interface::MoveGroupInterface &move_group,
                             const moveit::planning_interface::MoveGroupInterface::Plan &plan)
: nh_(nh),
  move_group_(move_group),
  plan_(plan),
  visual_tools_(frame),
  tf_listener_(nh_, ros::Duration(10)) {

  planning_scene_diff_publisher_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  prepMoveItVisualTools();
  addTableObstacle();
}

MoveitVisuals::~MoveitVisuals() {
  auto all_attached_objects = planning_scene_interface_.getAttachedObjects();
  auto all_objects = planning_scene_interface_.getObjects();

  for (auto it = begin(all_objects); it != end(all_objects); ++it) {
    removeObstacle(it->first);
  }
  // TODO remove attached objects as well
}

void MoveitVisuals::prepMoveItVisualTools() {
  visual_tools_.deleteAllMarkers();
  visual_tools_.loadRemoteControl();
  visual_tools_.trigger();
}

void MoveitVisuals::showPlannedPath() {
  visual_tools_.deleteAllMarkers();
  visual_tools_.publishTrajectoryLine(plan_.trajectory_, move_group_.getCurrentState()->getJointModelGroup("arm"));
  visual_tools_.trigger();
}

void MoveitVisuals::addTableObstacle() {
  constexpr double table_width = 65._cm;
  constexpr double table_length = 1.18_m;
  constexpr double table_height = 1._cm;
  sleep(1);

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_.getPlanningFrame();;
    collision_object.id = "table";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = table_length;
    primitive.dimensions[1] = table_width;
    primitive.dimensions[2] = table_height;

    geometry_msgs::PointStamped out_pt;
    geometry_msgs::PointStamped in_pt;
    in_pt.header.frame_id = "base_link";
    in_pt.header.stamp = ros::Time(0);
    in_pt.point.x = primitive.dimensions[0] * 0.5;
    in_pt.point.y = primitive.dimensions[1] * 0.5;
    in_pt.point.z = primitive.dimensions[2] * 0.5;

    // transform point
    try {
      tf_listener_.waitForTransform("root", in_pt.header.frame_id, in_pt.header.stamp, ros::Duration(2));
      tf_listener_.transformPoint("root", in_pt, out_pt);
    }
    catch (tf::TransformException &exception) {
      ROS_INFO_STREAM("MoveIt visuals: Transform failed. Why? - " << exception.what());
    }

    geometry_msgs::Pose pose;
    pose.position.x = out_pt.point.x;
    pose.position.y = out_pt.point.y;
    pose.position.z = out_pt.point.z - 0.05;
    pose.orientation.w = 1.0;

    collision_object.operation = collision_object.ADD;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_objects.push_back(collision_object);
  }

  ROS_SUCCESS("Added table as obstacle");

//  {
//    moveit_msgs::CollisionObject collision_object;
//    collision_object.header.frame_id = move_group_.getPlanningFrame();;
//    collision_object.id = "left-wall";
//
//    shape_msgs::SolidPrimitive primitive;
//    primitive.type = primitive.BOX;
//    primitive.dimensions.resize(3);
//    primitive.dimensions[0] = table_length;
//    primitive.dimensions[1] = table_height;
//    primitive.dimensions[2] = 1._m;
//
//    geometry_msgs::PointStamped out_pt;
//    geometry_msgs::PointStamped in_pt;
//    in_pt.header.frame_id = "base_link";
//    in_pt.header.stamp = ros::Time(0);
//    in_pt.point.x = primitive.dimensions[0] * 0.5;
//    in_pt.point.y = table_width + 10._cm + primitive.dimensions[1] * 0.5;
//    in_pt.point.z = primitive.dimensions[2] * 0.5;
//
//    // transform point
//    try {
//      tf_listener_.waitForTransform("root", in_pt.header.frame_id, in_pt.header.stamp, ros::Duration(2));
//      tf_listener_.transformPoint("root", in_pt, out_pt);
//    }
//    catch (tf::TransformException &exception) {
//      ROS_INFO_STREAM("MoveIt visuals: Transform failed. Why? - " << exception.what());
//    }
//
//    geometry_msgs::Pose pose;
//    pose.position.x = out_pt.point.x;
//    pose.position.y = out_pt.point.y;
//    pose.position.z = out_pt.point.z;
//    pose.orientation.w = 0.0;
//
//    collision_object.operation = collision_object.ADD;
//    collision_object.primitives.push_back(primitive);
//    collision_object.primitive_poses.push_back(pose);
//    collision_objects.push_back(collision_object);
//  }
//
//  ROS_SUCCESS("Added left Wall as obstacle");
//
//  {
//    moveit_msgs::CollisionObject collision_object;
//    collision_object.header.frame_id = move_group_.getPlanningFrame();;
//    collision_object.id = "right-wall";
//
//    shape_msgs::SolidPrimitive primitive;
//    primitive.type = primitive.BOX;
//    primitive.dimensions.resize(3);
//    primitive.dimensions[0] = table_length;
//    primitive.dimensions[1] = table_height;
//    primitive.dimensions[2] = 1._m;
//
//    geometry_msgs::PointStamped out_pt;
//    geometry_msgs::PointStamped in_pt;
//    in_pt.header.frame_id = "base_link";
//    in_pt.header.stamp = ros::Time(0);
//    in_pt.point.x = primitive.dimensions[0] * 0.5;
//    in_pt.point.y = -10._cm + primitive.dimensions[1] * 0.5;
//    in_pt.point.z = primitive.dimensions[2] * 0.5;
//
//    // transform point
//    try {
//      tf_listener_.waitForTransform("root", in_pt.header.frame_id, in_pt.header.stamp, ros::Duration(2));
//      tf_listener_.transformPoint("root", in_pt, out_pt);
//    }
//    catch (tf::TransformException &exception) {
//      ROS_INFO_STREAM("MoveIt visuals: Transform failed. Why? - " << exception.what());
//    }
//
//    geometry_msgs::Pose pose;
//    pose.position.x = out_pt.point.x;
//    pose.position.y = out_pt.point.y;
//    pose.position.z = out_pt.point.z;
//    pose.orientation.w = 0.0;
//
//    collision_object.operation = collision_object.ADD;
//    collision_object.primitives.push_back(primitive);
//    collision_object.primitive_poses.push_back(pose);
//    collision_objects.push_back(collision_object);
//  }
//
//  ROS_SUCCESS("Added right Wall as obstacle");
//
//  {
//    moveit_msgs::CollisionObject collision_object;
//    collision_object.header.frame_id = move_group_.getPlanningFrame();;
//    collision_object.id = "back-wall";
//
//    shape_msgs::SolidPrimitive primitive;
//    primitive.type = primitive.BOX;
//    primitive.dimensions.resize(3);
//    primitive.dimensions[0] = table_width + 20._cm;
//    primitive.dimensions[1] = table_height;
//    primitive.dimensions[2] = 1._m;
//
//    geometry_msgs::PointStamped out_pt;
//    geometry_msgs::PointStamped in_pt;
//    in_pt.header.frame_id = "base_link";
//    in_pt.header.stamp = ros::Time(0);
//    in_pt.point.x = -0.55_m + primitive.dimensions[0] * 0.5;
//    in_pt.point.y = table_width * 0.5; //26._cm + primitive.dimensions[1] * 0.5;
//    in_pt.point.z = primitive.dimensions[2] * 0.5;
//
//    // transform point
//    try {
//      tf_listener_.waitForTransform("root", in_pt.header.frame_id, in_pt.header.stamp, ros::Duration(2));
//      tf_listener_.transformPoint("root", in_pt, out_pt);
//    }
//    catch (tf::TransformException &exception) {
//      ROS_INFO_STREAM("MoveIt visuals: Transform failed. Why? - " << exception.what());
//    }
//
//    geometry_msgs::Pose pose;
//    pose.position.x = out_pt.point.x;
//    pose.position.y = out_pt.point.y;
//    pose.position.z = out_pt.point.z;
//    pose.orientation.x = std::sqrt(0.5);
//    pose.orientation.y = std::sqrt(0.5);
//
//    collision_object.operation = collision_object.ADD;
//    collision_object.primitives.push_back(primitive);
//    collision_object.primitive_poses.push_back(pose);
//    collision_objects.push_back(collision_object);
//  }
//
//  ROS_SUCCESS("Added back Wall as obstacle");
//
//  {
//    moveit_msgs::CollisionObject collision_object;
//    collision_object.header.frame_id = move_group_.getPlanningFrame();;
//    collision_object.id = "back-wall-obstacle";
//
//    shape_msgs::SolidPrimitive primitive;
//    primitive.type = primitive.BOX;
//    primitive.dimensions.resize(3);
//    primitive.dimensions[0] = table_width + 20._cm;
//    primitive.dimensions[1] = 20._cm;
//    primitive.dimensions[2] = 0.5_m;
//
//    geometry_msgs::PointStamped out_pt;
//    geometry_msgs::PointStamped in_pt;
//    in_pt.header.frame_id = "base_link";
//    in_pt.header.stamp = ros::Time(0);
//    in_pt.point.x = -0.55_m + primitive.dimensions[0] * 0.5;
//    in_pt.point.y = table_width * 0.5;//26._cm + primitive.dimensions[1] * 0.5;
//    in_pt.point.z = table_height * 0.5 + 10._cm;
//
//    // transform point
//    try {
//      tf_listener_.waitForTransform("root", in_pt.header.frame_id, in_pt.header.stamp, ros::Duration(2));
//      tf_listener_.transformPoint("root", in_pt, out_pt);
//    }
//    catch (tf::TransformException &exception) {
//      ROS_INFO_STREAM("MoveIt visuals: Transform failed. Why? - " << exception.what());
//    }
//
//    geometry_msgs::Pose pose;
//    pose.position.x = out_pt.point.x;
//    pose.position.y = out_pt.point.y;
//    pose.position.z = out_pt.point.z;
//    pose.orientation.x = std::sqrt(0.5);
//    pose.orientation.y = std::sqrt(0.5);
//
//    collision_object.operation = collision_object.ADD;
//    collision_object.primitives.push_back(primitive);
//    collision_object.primitive_poses.push_back(pose);
//    collision_objects.push_back(collision_object);
//  }
//
//  ROS_SUCCESS("Added back Wall obstacle as obstacle");
//
//  {
//    moveit_msgs::CollisionObject collision_object;
//    collision_object.header.frame_id = move_group_.getPlanningFrame();;
//    collision_object.id = "top-wall";
//
//    shape_msgs::SolidPrimitive primitive;
//    primitive.type = primitive.BOX;
//    primitive.dimensions.resize(3);
//    primitive.dimensions[0] = table_length * 0.5;
//    primitive.dimensions[1] = table_width;
//    primitive.dimensions[2] = table_height;
//
//    geometry_msgs::PointStamped out_pt;
//    geometry_msgs::PointStamped in_pt;
//    in_pt.header.frame_id = "base_link";
//    in_pt.header.stamp = ros::Time(0);
//    in_pt.point.x = primitive.dimensions[0] * 0.5;
//    in_pt.point.y = primitive.dimensions[1] * 0.5;
//    in_pt.point.z = 90._cm + primitive.dimensions[2] * 0.5;
//
//    // transform point
//    try {
//      tf_listener_.waitForTransform("root", in_pt.header.frame_id, in_pt.header.stamp, ros::Duration(2));
//      tf_listener_.transformPoint("root", in_pt, out_pt);
//    }
//    catch (tf::TransformException &exception) {
//      ROS_INFO_STREAM("MoveIt Visuals: Transform failed. Why? - " << exception.what());
//    }
//
//    geometry_msgs::Pose pose;
//    pose.position.x = out_pt.point.x;
//    pose.position.y = out_pt.point.y;
//    pose.position.z = out_pt.point.z - 0.05;
//    pose.orientation.w = 1.0;
//
//    collision_object.operation = collision_object.ADD;
//    collision_object.primitives.push_back(primitive);
//    collision_object.primitive_poses.push_back(pose);
//    collision_objects.push_back(collision_object);
//  }
//
//  ROS_SUCCESS("Added top wall as obstacle");

  planning_scene_interface_.addCollisionObjects(collision_objects);
  sleep(1);
}

void MoveitVisuals::addObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal) {
  const auto& pose_goal = goal->pose_goal;
  const auto& box = goal->bounding_box;

  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "jaco_link_hand";
  attached_object.object.header.frame_id = move_group_.getPlanningFrame();
  attached_object.object.id = box.description;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = box.dimensions.x;
  primitive.dimensions[1] = box.dimensions.y;
  primitive.dimensions[2] = box.dimensions.z;

  geometry_msgs::PointStamped out_pt;
  geometry_msgs::PointStamped in_pt;

  if (box.point.z == 0.0)
    ROS_ERROR("Is it acutally the centroid? Height is %f", box.point.z);

  in_pt.header = box.header;
  in_pt.point = box.point;

  // transform point
  try {
    tf_listener_.waitForTransform("root", box.header.frame_id, box.header.stamp, ros::Duration(1));
    tf_listener_.transformPoint("root", in_pt, out_pt);
  }
  catch (tf::TransformException &exception) {
    ROS_INFO_STREAM("MoveIt visuals: Transform failed. Why? - " << exception.what());
  }

  geometry_msgs::Pose pose;
  pose.position.x = out_pt.point.x;
  pose.position.y = out_pt.point.y;
  pose.position.z = out_pt.point.z;
  pose.orientation.w = 1.0;

  attached_object.object.operation = attached_object.object.ADD;
  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  attached_object.touch_links = std::vector<std::string>{ "base_link",
                                                          "jaco_link_hand",
                                                          "jaco_link_finger_1",
                                                          "jaco_link_finger_2",
                                                          "jaco_link_finger_3",
                                                          "jaco_link_finger_tip_1",
                                                          "jaco_link_finger_tip_2",
                                                          "jaco_link_finger_tip_3" };

  planning_scene_.world.collision_objects.push_back(attached_object.object);
  planning_scene_.is_diff = true;
  planning_scene_diff_publisher_.publish(planning_scene_);

  to_be_attached[box.description] = attached_object;

  ROS_SUCCESS("Added Object " << box.description);
}

void MoveitVisuals::attachObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal) {
  const jaco_manipulation::BoundingBox &box = goal->bounding_box;

  auto all_objects = planning_scene_interface_.getObjects();
  if (all_objects.find(box.description) == end(all_objects)) {
    ROS_ERROR_STREAM("Can't attach object " << box.description << ". Wasn't created!");
    return;
  }

  if (to_be_attached.find((box.description)) == end(to_be_attached)) {
    ROS_ERROR_STREAM("Object to be attached wasn't saved");
    return;
  }

  /* REMOVE object message*/
  moveit_msgs::CollisionObject remove_object;
  remove_object.id = box.description;
  remove_object.header.frame_id = move_group_.getPlanningFrame();
  remove_object.operation = remove_object.REMOVE;

  planning_scene_.world.collision_objects.clear();
  planning_scene_.world.collision_objects.push_back(remove_object);
  planning_scene_.robot_state.attached_collision_objects.push_back(to_be_attached.at(box.description));

  planning_scene_diff_publisher_.publish(planning_scene_);

  ROS_STATUS("Attached obstacle " << box.description);
  showStatus();
}

void MoveitVisuals::detachObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal) {
  const jaco_manipulation::BoundingBox &box = goal->bounding_box;

  if (to_be_attached.find(box.description) == end(to_be_attached)) {
    ROS_ERROR("Object to be detached wasn't saved as attached object!");
    return;
  }

  auto &attached_objects = planning_scene_.robot_state.attached_collision_objects;
  if (std::none_of(begin(attached_objects), end(attached_objects), [&box](const auto &obj) {
    return obj.object.id == box.description;
  })) {
    ROS_ERROR_STREAM("Not attached: " << box.description);
    return;
  }

  /* DETACH object message*/
  moveit_msgs::AttachedCollisionObject object_to_be_detached;
  for (const auto& obj: planning_scene_.robot_state.attached_collision_objects) {
    if (obj.object.id == box.description)
      object_to_be_detached = obj;
  }

  moveit_msgs::AttachedCollisionObject detach_object;
  detach_object.object.id = box.description;
  detach_object.link_name = "jaco_link_hand";
  detach_object.object.operation = object_to_be_detached.object.REMOVE;

  planning_scene_.robot_state.attached_collision_objects.clear();
  planning_scene_.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene_.robot_state.is_diff = true;
  planning_scene_diff_publisher_.publish(planning_scene_);

  addObstacle(goal);

  ROS_STATUS("Detached obstacle " << box.description);
  showStatus();
}

void MoveitVisuals::removeObstacle(const std::string id) {
//  std::vector<std::string> object_ids;
//  object_ids.push_back(id);
//  planning_scene_interface_.removeCollisionObjects(object_ids);
  moveit_msgs::CollisionObject remove_object;
  remove_object.id = id;
  remove_object.header.frame_id = move_group_.getPlanningFrame();
  remove_object.operation = remove_object.REMOVE;

  planning_scene_.robot_state.attached_collision_objects.clear();
  planning_scene_.world.collision_objects.clear();
  planning_scene_.world.collision_objects.push_back(remove_object);
  planning_scene_diff_publisher_.publish(planning_scene_);
  ROS_SUCCESS("Removed Object " << id << " from planning scene");
}

void MoveitVisuals::wipeKinectObstacles() {
  auto all_objects = planning_scene_interface_.getObjects();
  for (auto it = begin(all_objects); it != end(all_objects); ++it) {
    if (it->first == "table" || it->first == "wall") continue;
    removeObstacle(it->first);
  }
}

void MoveitVisuals::showStatus() {
  auto all_attached_objects = planning_scene_interface_.getAttachedObjects();
  ROS_STATUS("All Attached Objects: ");
  for (auto it = begin(all_attached_objects); it != end(all_attached_objects); ++it) {
    ROS_STATUS(it->second);
  }

  ROS_STATUS("All Objects: ");
  auto contains = [](const std::string& label, const std::string& target) { return label.find(target) != std::string::npos; };
  auto all_objects = planning_scene_interface_.getObjects();
  for (auto it = begin(all_objects); it != end(all_objects); ++it) {
    if (it->first == "table" || contains(it->first, "wall")) continue;
    ROS_STATUS(it->second);
  }
}

unsigned long MoveitVisuals::numOfObstacles() {
  return planning_scene_interface_.getObjects().size();
}

const std::vector<jaco_manipulation::BoundingBox> &MoveitVisuals::getObstacles() {
  obstacles_.clear();
  auto all_objects = planning_scene_interface_.getObjects();
  for (auto it = begin(all_objects); it != end(all_objects); ++it) {
    const auto& collision_object = it->second;
    jaco_manipulation::BoundingBox box;
    box.description = collision_object.id;
    box.header = collision_object.header;
    if (collision_object.primitives[collision_object.primitives.size()-1].dimensions.size() >= 3)
    {
      box.dimensions.x = collision_object.primitives[collision_object.primitives.size()-1].dimensions[0];
      box.dimensions.y = collision_object.primitives[collision_object.primitives.size()-1].dimensions[1];
      box.dimensions.z = collision_object.primitives[collision_object.primitives.size()-1].dimensions[2];
    } else {
      ROS_ERROR_STREAM("Can't add collisition box information to debug publisher: " << box.description);
    }
    box.point = collision_object.primitive_poses[collision_object.primitive_poses.size()-1].position;
    obstacles_.push_back(box);
  }
  return obstacles_;
}
