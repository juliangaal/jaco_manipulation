//
// Created by chitt on 8/13/18.
//

#include <jaco_manipulation/visuals/moveit_visuals.h>
#include <jaco_manipulation/server/jaco_manipulation_server.h>

using namespace jaco_manipulation::visuals;

MoveitVisuals::MoveitVisuals(ros::NodeHandle &nh, const std::string frame,
                             moveit::planning_interface::MoveGroupInterface &move_group,
                             const moveit::planning_interface::MoveGroupInterface::Plan &plan)
: nh_(nh),
  move_group_(move_group),
  plan_(plan),
  visual_tools_(frame),
  tf_listener_(nh_, ros::Duration(10)) {
    prepMoveItVisualTools();
    addTableObstacle();
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
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_.getPlanningFrame();;
    collision_object.id = "table";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.18;
    primitive.dimensions[1] = 0.62;
    primitive.dimensions[2] = -0.001;

    geometry_msgs::PointStamped out_pt;
    geometry_msgs::PointStamped in_pt;
    in_pt.header.frame_id = "base_link";
    in_pt.header.stamp = ros::Time::now();
    in_pt.point.x = primitive.dimensions[0] * 0.5;
    in_pt.point.y = primitive.dimensions[1] * 0.5;
    in_pt.point.z = primitive.dimensions[2] * 0.5;

    // transform point
    try {
      tf_listener_.waitForTransform("root", in_pt.header.frame_id, in_pt.header.stamp, ros::Duration(2));
      tf_listener_.transformPoint("root", in_pt, out_pt);
    }
    catch (tf::TransformException &exception) {
      ROS_INFO_STREAM("Transform failed. Why? - " << exception.what());
    }

    ROS_INFO("Table Transfrm: \"%s\" (%f,%f,%f) -> \"%s\" (%f,%f,%f)",
             in_pt.header.frame_id.c_str(),
             in_pt.point.x,
             in_pt.point.y,
             in_pt.point.z,
             out_pt.header.frame_id.c_str(),
             out_pt.point.x,
             out_pt.point.y,
             out_pt.point.z);

    geometry_msgs::Pose pose;
    pose.position.x = out_pt.point.x;
    pose.position.y = out_pt.point.y;
    pose.position.z = out_pt.point.z - 0.01;
    pose.orientation.w = 1.0;

    collision_object.operation = collision_object.ADD;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_objects.push_back(collision_object);
  }

  {
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group_.getPlanningFrame();;
    collision_object.id = "wall";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.18;
    primitive.dimensions[1] = 0.01;
    primitive.dimensions[2] = 0.31;

    geometry_msgs::PointStamped out_pt;
    geometry_msgs::PointStamped in_pt;
    in_pt.header.frame_id = "base_link";
    in_pt.header.stamp = ros::Time::now();
    in_pt.point.x = primitive.dimensions[0] * 0.5;
    in_pt.point.y = 0.62 + primitive.dimensions[1] * 0.5;
    in_pt.point.z = primitive.dimensions[2] * 0.5;

    // transform point
    try {
      tf_listener_.waitForTransform("root", in_pt.header.frame_id, in_pt.header.stamp, ros::Duration(2));
      tf_listener_.transformPoint("root", in_pt, out_pt);
    }
    catch (tf::TransformException &exception) {
      ROS_INFO_STREAM("Transform failed. Why? - " << exception.what());
    }

    ROS_INFO("Wall Transform: \"%s\" (%f,%f,%f) -> \"%s\" (%f,%f,%f)",
             in_pt.header.frame_id.c_str(),
             in_pt.point.x,
             in_pt.point.y,
             in_pt.point.z,
             out_pt.header.frame_id.c_str(),
             out_pt.point.x,
             out_pt.point.y,
             out_pt.point.z);

    geometry_msgs::Pose pose;
    pose.position.x = out_pt.point.x;
    pose.position.y = out_pt.point.y;
    pose.position.z = out_pt.point.z;
    pose.orientation.w = 0.0;

    collision_object.operation = collision_object.ADD;
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose);
    collision_objects.push_back(collision_object);
  }

  planning_scene_interface_.addCollisionObjects(collision_objects);
}

void MoveitVisuals::addObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal) {
  const auto& pose_goal = goal->pose_goal;
  const auto& box = goal->bounding_box;

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group_.getPlanningFrame();
  collision_object.id = box.description;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = box.dimensions.x;
  primitive.dimensions[1] = box.dimensions.y;
  primitive.dimensions[2] = box.dimensions.z;

  geometry_msgs::PointStamped out_pt;
  geometry_msgs::PointStamped in_pt;
  in_pt.header = box.header;
  in_pt.point = box.point;

  // transform point
  try {
    tf_listener_.waitForTransform("root", box.header.frame_id, box.header.stamp, ros::Duration(1));
    tf_listener_.transformPoint("root", in_pt, out_pt);
  }
  catch (tf::TransformException &exception) {
    ROS_INFO_STREAM("Transform failed. Why? - " << exception.what());
  }

  geometry_msgs::Pose pose;
  pose.position.x = out_pt.point.x;
  pose.position.y = out_pt.point.y;
  pose.position.z = out_pt.point.z;
  pose.orientation.w = 1.0;

  collision_object.operation = collision_object.ADD;
  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(pose);

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  planning_scene_interface_.addCollisionObjects(collision_objects);

  ROS_STATUS("Added Object " << box.description);
}

void MoveitVisuals::attachObstacle(const jaco_manipulation::BoundingBox &box) {
  auto all_objects = planning_scene_interface_.getAttachedObjects();
  if (all_objects.find(box.description) != all_objects.end()) {
    ROS_WARN_STREAM("Object " << box.description << " is already attached");
    return;
  }

  ROS_STATUS("Attaching obstacle " << box.description);
  move_group_.attachObject(box.description);
}

void MoveitVisuals::detachObstacle(const jaco_manipulation::BoundingBox &box) {
  auto all_objects = planning_scene_interface_.getAttachedObjects();
  if (all_objects.find(box.description) == all_objects.end()) {
    ROS_WARN_STREAM("Object " << box.description << " is not attached! Can't detach");
    return;
  }

  ROS_STATUS("Detaching obstacle " << box.description);
  move_group_.detachObject(box.description);
}

void MoveitVisuals::removeObstacle(const jaco_manipulation::BoundingBox &box) {
  detachObstacle(box);
  std::vector<std::string> object_ids;
  object_ids.push_back(box.description);
  planning_scene_interface_.removeCollisionObjects(object_ids);
  ROS_STATUS("Removed Object " << box.description << " from planning scene");
}
