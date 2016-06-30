/*
  Copyright (C) 2015  Chittaranjan Srinivas Swaminathan

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
#include <jaco_manipulation/grasp_pose_generator.h>

geometry_msgs::Point operator + (geometry_msgs::Point p1, geometry_msgs::Point p2) {
  geometry_msgs::Point result;
  result.x = p1.x + p2.x;
  result.y = p1.y + p2.y;
  result.z = p1.z + p2.z;
  return result;
}

namespace jaco_manipulation {

GraspPoseGenerator::GraspPoseGenerator()
{
  tf_listener_ = new tf::TransformListener(n_,ros::Duration(10));
  server_ = n_.advertiseService ("generate_grasp_poses", &GraspPoseGenerator::serverCB, this);

  ROS_INFO("Server Started");

}

GraspPoseGenerator::~GraspPoseGenerator()
{
  ROS_INFO("GraspPoseGenerator Shutting down...");

  if(tf_listener_)
    delete tf_listener_;
}

geometry_msgs::PointStamped GraspPoseGenerator::transformPointToBaseLink(const geometry_msgs::PointStamped& in_pt)
{
  geometry_msgs::PointStamped out_pt;

  try
  {
    tf_listener_->waitForTransform("root", in_pt.header.frame_id, in_pt.header.stamp, ros::Duration(1));
    tf_listener_->transformPoint("root", in_pt, out_pt);
  }
  catch(tf::TransformException& exception)
  {
    ROS_INFO("Transform failed. Why? - %s", exception.what());
  }

  return out_pt;
}

bool GraspPoseGenerator::serverCB(jaco_manipulation::GenerateGraspPosesRequest& _request,
    jaco_manipulation::GenerateGraspPosesResponse& _response)
{

  geometry_msgs::PointStamped pt = _request.object_location;

  ROS_INFO("GPG: %f, %f, %f", pt.point.x, pt.point.y, pt.point.z);
  pt = transformPointToBaseLink(pt);

  ROS_INFO("GPG: Transformed to: %f, %f, %f", pt.point.x, pt.point.y, pt.point.z);

  tf::Vector3 p (
      pt.point.x,
      pt.point.y,
      pt.point.z
  );

  geometry_msgs::PointStamped pt_o; // Same frame.

  pt_o.header.stamp = ros::Time::now();
  pt_o.header.frame_id = "root";
  pt_o.point.x = 0.0;
  pt_o.point.y = 0.0;
  pt_o.point.z = 1.0;

  tf::Vector3 view;
  view.setValue (
      p.x() - pt_o.point.x,
      p.y() - pt_o.point.y,
      p.z() - pt_o.point.z);

  // Orientation:
  //  - X points to the thumb.
  //  - Y is perpendicular to the grasp plane.
  //  - Z is the wrist rotation axis, and should point outward
  //    the camera.

  // Let's jiggle it a bit
  tf::Matrix3x3 rotate45y (0.707, 0, -0.707, 0, 1, 0, 0.707, 0, 0.707);
  tf::Matrix3x3 rotate50y (0.64278, 0, -0.766, 0, 1, 0, 0.766, 0, 0.64278);
  tf::Matrix3x3 rotate25y (0.906, 0, -0.423, 0, 1, 0, 0.423, 0, 0.906);
  tf::Matrix3x3 rotate10z (0.9848, 0.17365, 0, -0.17365, 0.9848, 0, 0, 0, 1);
  tf::Matrix3x3 rotatex (1, 0, 0, 0, 0.9848, 0.17365, 0, -0.17365, 0.9848);

  tf::Matrix3x3 rotatey (0.9848, 0, 0.17365, 0, 1, 0, -0.17365, 0, 0.9848);
  tf::Matrix3x3 rotate25z (0.906, -0.423, 0, 0.423, 0.906, 0, 0, 0, 1);


  // The Vectors for the top grasp
  tf::Vector3 abs_z (0.0, 0.0, 1.0);
  tf::Vector3 oz_top = view;
  tf::Vector3 ox_top = abs_z.cross(view);

  oz_top = rotatey*oz_top;
  tf::Vector3 oy_top = oz_top.cross(ox_top);

  // Simple grasp vectors
  tf::Vector3 oy_1 = abs_z * -1;
  tf::Vector3 ox_1 = (oy_1.cross(view).normalized());
  tf::Vector3 oz_1 = ox_1.cross(oy_1);

  // Second simple grasp
  tf::Vector3 ox_2;
  tf::Vector3 oy_2;
  tf::Vector3 oz_2;

  // Third
  tf::Vector3 ox_3;
  tf::Vector3 oy_3;
  tf::Vector3 oz_3;

  // A 25 degree rotation about the global z
  oz_2 = rotate25z * oz_1;
  ox_2 = rotate25z * ox_1;
  oy_2 = oy_1;

  // A 50 degree rotation about the global z

  oz_3 = rotate25z * oz_2;
  ox_3 = rotate25z * ox_2;
  oy_3 = oy_2;

  // Convert this into a quaternion from the basis matrix.
  tf::Matrix3x3 basis_top(
      ox_top.x(), oy_top.x(), oz_top.x(),
      ox_top.y(), oy_top.y(), oz_top.y(),
      ox_top.z(), oy_top.z(), oz_top.z() );

  tf::Matrix3x3 basis_1(
      ox_1.x(), oy_1.x(), oz_1.x(),
      ox_1.y(), oy_1.y(), oz_1.y(),
      ox_1.z(), oy_1.z(), oz_1.z() );

  tf::Matrix3x3 basis_2(
      ox_2.x(), oy_2.x(), oz_2.x(),
      ox_2.y(), oy_2.y(), oz_2.y(),
      ox_2.z(), oy_2.z(), oz_2.z() );

  tf::Matrix3x3 basis_3(
      ox_3.x(), oy_3.x(), oz_3.x(),
      ox_3.y(), oy_3.y(), oz_3.y(),
      ox_3.z(), oy_3.z(), oz_3.z() );

  tf::Vector3 target_p = p;

  // We have the positions and rotations of the Tool frame.
  // We convert it to position and rotation of the Hand frame (Frame 6).

  tf::Matrix3x3 FromToolTo6 (
      1, 0, 0,
      0, -1, 0,
      0, 0,	-1
  );

  basis_top = basis_top * FromToolTo6;
  basis_1= basis_1 * FromToolTo6;
  basis_2= basis_2* FromToolTo6;
  basis_3 = basis_3* FromToolTo6;

  // We need to finally incorporate the distance between the tool frame and frame no. 6
  tf::Vector3 offset(0.0, 0.0, 0.15);
  tf::Vector3 offset1 = (basis_1 * offset) - (basis_top * tf::Vector3(0.0,0.0,0.0));
  tf::Vector3 offset2 = (basis_2 * offset) - (basis_top * tf::Vector3(0.0,0.0,0.0));
  tf::Vector3 offset3 = (basis_3 * offset) - (basis_top * tf::Vector3(0.0,0.0,0.0));
  tf::Vector3 offset_top = (basis_top * offset) - (basis_top * tf::Vector3(0.0,0.0,0.0));

  // Verbose - bad programming - but this should be enough for the simple case

  // Header assignment. We will later change the frame id alone.
  target_pose_top_.header = _request.object_location.header;
  target_pose_1_.header = _request.object_location.header;
  target_pose_2_.header = _request.object_location.header;
  target_pose_3_.header = _request.object_location.header;

  geometry_msgs::Point _point, _offset1, _offset2, _offset3, _offset_top;

  tf::pointTFToMsg(target_p, _point);
  tf::pointTFToMsg(offset1, _offset1);
  tf::pointTFToMsg(offset2, _offset2);
  tf::pointTFToMsg(offset3, _offset3);
  tf::pointTFToMsg(offset_top, _offset_top);

  target_pose_top_.pose.position = _point + _offset_top;
  target_pose_1_.pose.position = _point + _offset1;
  target_pose_2_.pose.position = _point + _offset2;
  target_pose_3_.pose.position = _point + _offset3;

  tf::Quaternion q_1, q_2, q_3, q_top;

  // Top
  basis_top.getRotation(q_top);
  q_top = q_top.normalized();
  tf::quaternionTFToMsg(q_top, target_pose_top_.pose.orientation);

  // 1
  basis_1.getRotation(q_1);
  q_1 = q_1.normalized();
  tf::quaternionTFToMsg(q_1, target_pose_1_.pose.orientation);

  // 2
  basis_2.getRotation(q_2);
  q_2 = q_2.normalized();
  tf::quaternionTFToMsg(q_2, target_pose_2_.pose.orientation);

  // 3
  basis_3.getRotation(q_3);
  q_3 = q_3.normalized();
  tf::quaternionTFToMsg(q_3, target_pose_3_.pose.orientation);


  _response.grasp_poses.resize(4);
  _response.pregrasp_poses.resize(4);

  // First the grasp pose. Then the pregrasp pose
  target_pose_top_.pose.position.z += 0.02;
  target_pose_top_.pose.position.y -= 0.025;
  _response.grasp_poses[3] = target_pose_top_;
  target_pose_top_.pose.position.z += 0.03;
  _response.pregrasp_poses[3] = target_pose_top_;


  // First the grasp pose. Then the pregrasp pose
  target_pose_1_.pose.position.y -= 0.020;
  _response.grasp_poses[1] = target_pose_1_;
  target_pose_1_.pose.position.y += 0.07;
  _response.pregrasp_poses[1] = target_pose_1_;

  // First the grasp pose. Then the pregrasp pose
  target_pose_2_.pose.position.y -= 0.020;
  target_pose_2_.pose.position.x += 0.020;
  _response.grasp_poses[2] = target_pose_2_;
  target_pose_2_.pose.position.y += 0.070;
  target_pose_2_.pose.position.x -= 0.070;
  _response.pregrasp_poses[2] = target_pose_2_;

  // First the grasp pose. Then the pregrasp pose
  target_pose_3_.pose.position.y -= 0.022;
  target_pose_3_.pose.position.x += 0.022;
  _response.grasp_poses[0] = target_pose_3_;
  target_pose_3_.pose.position.y += 0.070;
  target_pose_3_.pose.position.x -= 0.070;
  _response.pregrasp_poses[0] = target_pose_3_;

  _response.grasp_poses[0].header.frame_id = "root";
  _response.grasp_poses[1].header.frame_id = "root";
  _response.grasp_poses[2].header.frame_id = "root";
  _response.grasp_poses[3].header.frame_id = "root";

  _response.pregrasp_poses[0].header.frame_id = "root";
  _response.pregrasp_poses[1].header.frame_id = "root";
  _response.pregrasp_poses[2].header.frame_id = "root";
  _response.pregrasp_poses[3].header.frame_id = "root";
  // Set the number and despatch
  return true;
}

float GraspPoseGenerator::dist3D(const geometry_msgs::Point p1, const geometry_msgs::Point p2) {

  float dx = p2.x - p1.x;
  float dy = p2.y - p1.y;
  float dz = p2.z - p1.z;

  return sqrt((float)(dx * dx + dy * dy + dz * dz));
}

}

