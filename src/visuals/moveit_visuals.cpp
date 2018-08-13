//
// Created by chitt on 8/13/18.
//

#include <jaco_manipulation/visuals/moveit_visuals.h>

using namespace jaco_manipulation::visuals;

MoveitVisuals::MoveitVisuals(ros::NodeHandle &nh, const std::string frame,
                             moveit::planning_interface::MoveGroupInterface &move_group,
                             const moveit::planning_interface::MoveGroupInterface::Plan &plan)
: nh_(nh), move_group_(move_group), plan_(plan), visual_tools_(frame), tf_listener_(nh_, ros::Duration(10)) {
    planning_scene_diff_publisher = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
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
  moveit_msgs::AttachedCollisionObject collision_object;
  collision_object.object.header.frame_id = move_group_.getPlanningFrame();
  collision_object.object.id = "table";

  geometry_msgs::Pose pose;
  pose.position.x = 0.0;
  pose.position.y = 0.0;
  pose.position.z = -0.026;
  pose.orientation.w = 1.0;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.5;
  primitive.dimensions[1] = 0.62;
  primitive.dimensions[2] = 0.001;

  collision_object.object.primitives.push_back(primitive);
  collision_object.object.primitive_poses.push_back(pose);
  collision_object.object.operation = collision_object.object.ADD;

  planning_scene_.world.collision_objects.push_back(collision_object.object);
  planning_scene_.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene_);

  visual_tools_.trigger();
}

void MoveitVisuals::addObstacle(const jaco_manipulation::PlanAndMoveArmGoalConstPtr &goal) {
  const auto& pose_goal = goal->pose_goal;
  const auto& box = goal->bounding_box;

  moveit_msgs::AttachedCollisionObject collision_object;
  collision_object.object.header.frame_id = move_group_.getPlanningFrame();
  collision_object.object.id = goal->bounding_box.description;

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
  ROS_INFO("Transfrm: \"%s\" (%f,%f,%f) -> \"%s\" (%f,%f,%f)",
           box.header.frame_id.c_str(),
           in_pt.point.x,
           in_pt.point.y,
           in_pt.point.z,
           "root",
           out_pt.point.x,
           out_pt.point.y,
           out_pt.point.z);

  geometry_msgs::Pose pose;
  pose.position.x = out_pt.point.x;
  pose.position.y = out_pt.point.y;
  pose.position.z = out_pt.point.z;
  pose.orientation.w = 1.0;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = box.dimensions.x;
  primitive.dimensions[1] = box.dimensions.y;
  primitive.dimensions[2] = box.dimensions.z;

  collision_object.object.primitives.push_back(primitive);
  collision_object.object.primitive_poses.push_back(pose);
  collision_object.object.operation = collision_object.object.ADD;
  collision_object.touch_links = std::vector<std::string>{"jaco_link_hand",
                                                          "jaco_link_finger_1",
                                                          "jaco_link_finger_2",
                                                          "jaco_link_finger_3",
                                                          "jaco_link_finger_tip_1",
                                                          "jaco_link_finger_tip_2",
                                                          "jaco_link_finger_tip_3",
                                                          "jaco_small_ring_cover_3"};

  planning_scene_.world.collision_objects.push_back(collision_object.object);
  planning_scene_.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene_);

  visual_tools_.trigger();
}
