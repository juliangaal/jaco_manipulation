//
// Created by chitt on 8/6/18.
//

#ifndef PROJECT_KINECT_GOAL_HPP
#define PROJECT_KINECT_GOAL_HPP

#include "pose_goal.h"
#include <jaco_manipulation/grasps/grasp_pose_generator.h>
#include <jaco_manipulation/goals/goal_input.h>

namespace jaco_manipulation {
namespace goals {

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
  explicit KinectGoal(const goal_input::LimitedPose &grasp_pose_goal,
                      jaco_manipulation::grasps::GraspType grasp,
                      const std::string &description = "grasp goal");

  /**
   * Constructor
   * @param bounding_box_goal bounding box to drop something at
   * @param description descritpion with additional info
   */
  explicit KinectGoal(const goal_input::BoundingBox &bounding_box_goal,
                      jaco_manipulation::grasps::GraspType grasp,
                      const std::string &description = "grasp box goal");

  /**
   * default destructor
   */
  virtual ~KinectGoal() override = default;

  /**
   * get created goal
   */
  virtual jaco_manipulation::PlanAndMoveArmGoal goal() const override;

  /**
   * Returns requested grasp in string form
   */
  std::string requestedOrientation();

 protected:

  /**
   * protected default constructor
  */
  KinectGoal() = default;

  /**
   * Default dropping offset: tiny lift (5cm) + distance from jaco_lowest point (is marked on robot) to jaco palm (6cm)
   */
  constexpr static double dropping_offset_ = 0.16;

  /**
   * Saves which grasp was requested
   */
  const jaco_manipulation::grasps::GraspType requested_grasp_;

 private:

  jaco_manipulation::grasps::GraspPoseGenerator grasp_orientation_generator_{};

//  /**
//   * Adjusts the Pose to Center of Object
//   * @param bounding_box bounding box to center pose around
//  */
//  void adjustPoseToCenterOfObject(const goal_input::BoundingBox &bounding_box);

  /**
   * Adjusts gripper pose orientation and position for type of grasp
   * @param grasp type of grasp
   * @param box Bounding box to be adjusted around
   */
  void adjustPose(jaco_manipulation::grasps::GraspType grasp,
                  const jaco_manipulation::goals::goal_input::BoundingBox &box);
};
} // namespace goals
} // namespace jaco_manipulation

#endif //PROJECT_KINECT_GOAL_HPP
