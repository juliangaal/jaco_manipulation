//
// Created by chitt on 8/6/18.
//

#ifndef PROJECT_KINECT_GOAL_HPP
#define PROJECT_KINECT_GOAL_HPP

#include <jaco_manipulation/client/goals/pose_goal.h>
#include <jaco_manipulation/client/grasps/grasp_orientation_generator.h>

namespace jaco_manipulation {
namespace client {
namespace goals {
namespace kinect_goal_definitions {

/**
 * Object to grasp, defined by bounding box
 */
struct BoundingBox {
  std::string description;
  std::string type;
  double x;
  double y;
  double z;

  double height;
  double width;
  double length;
};

/**
 * additional layer of security: grasp_helper::GraspPose allows only to change properties on pose that are not dangerous
 */
struct LimitedPose {
  double x;
  double y;
  double z;
  double rotation;
};

}

namespace objects {

/*!
 * ObjectGoal
 * Represent a severely limited pose for grasping objects
 * Limited in height, and rotation around x and y axis
 */
class KinectGoal: public PoseGoal {
 public:

  /**
   * Constructor
   * @param grasp_pose_goal grasp pose goal
   * @param description descritpion with additional info
   */
  explicit KinectGoal(const kinect_goal_definitions::LimitedPose &grasp_pose_goal,
                      jaco_manipulation::client::grasps::GraspType grasp,
                      const std::string &description = "grasp goal");

  /**
   * Constructor
   * @param bounding_box_goal bounding box to drop something at
   * @param description descritpion with additional info
   */
  explicit KinectGoal(const kinect_goal_definitions::BoundingBox &bounding_box_goal,
                      jaco_manipulation::client::grasps::GraspType grasp,
                      const std::string &description = "grasp box goal");

  /**
   * default destructor
   */
  virtual ~KinectGoal() override = default;

  /**
   * get created goal
   */
  virtual jaco_manipulation::PlanAndMoveArmGoal goal() const override;

 protected:

  /**
   * protected default constructor
  */
  KinectGoal() = default;

  /**
   * Default dropping offset: tiny lift (5cm) + distance from jaco_lowest point (is marked on robot) to jaco palm (6cm)
   */
  constexpr static double dropping_offset_ = 0.16;

 private:

  jaco_manipulation::client::grasps::GraspOrientationGenerator grasp_orientation_generator_{};

  /**
   * Adjusts the Pose to Center of Object
   * @param bounding_box bounding box to center pose around
  */
  void adjustPoseToCenterOfObject(const kinect_goal_definitions::BoundingBox &bounding_box);

  /**
   * Adjusts gripper pose orientation to absolute orientation relative to root
   */
  void adjustOrientation(jaco_manipulation::client::grasps::GraspType grasp);
};
} // namespace objects
} // namespace goals
} // namespace client
} // namespace jaco_manipulation

#endif //PROJECT_KINECT_GOAL_HPP
