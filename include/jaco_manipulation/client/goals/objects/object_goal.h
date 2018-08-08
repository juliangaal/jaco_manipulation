//
// Created by chitt on 8/6/18.
//

#ifndef PROJECT_OBJECTGOAL_HPP
#define PROJECT_OBJECTGOAL_HPP
#include "jaco_manipulation/client/goals/pose_goal.h"

namespace jaco_manipulation {
namespace client {
namespace goals {
namespace kinect_goal {

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
class ObjectGoal: public PoseGoal {
 public:

  /**
   * Constructor
   * @param grasp_pose_goal grasp pose goal
   * @param description descritpion with additional info
   */
  explicit ObjectGoal(const kinect_goal::LimitedPose &grasp_pose_goal, const std::string &description = "grasp goal");

  /**
   * Constructor
   * @param bounding_box_goal bounding box to drop something at
   * @param description descritpion with additional info
   */
  explicit ObjectGoal(const kinect_goal::BoundingBox &bounding_box_goal, const std::string &description = "grasp box goal");

  /**
   * default destructor
   */
  virtual ~ObjectGoal() override = default;

  /**
   * get created goal
   */
  virtual jaco_manipulation::PlanAndMoveArmGoal goal() const override;

 protected:

  /**
   * protected default constructor
  */
  ObjectGoal() = default;

  /**
   * Adjusts the Pose to Center of Object
   * @param bounding_box bounding box to center pose around
   */
  void adjustPoseToCenterOfObject(const kinect_goal::BoundingBox &bounding_box);

  /**
   * Default rotation for posw
  */
  constexpr static double default_orientation_ = 0.674663;

  /**
   * Default rotation value x axis
  */
  constexpr static double default_rot_x_ = 0.033245;

  /**
   * Default rotation value y axis
  */
  constexpr static double default_rot_y_ = -0.003230;

  /**
   * Default rotation value z axis
  */
  constexpr static double default_rot_z_ = -0.737370;

  /**
   * Default dropping offset: tiny lift (5cm) + distance from jaco_lowest point (is marked on robot) to jaco palm (6cm)
   */
  constexpr static double dropping_offset_ = 0.02;
};

} // namespace objects
} // namespace goals
} // namespace client
} // namespace jaco_manipulation

#endif //PROJECT_OBJECTGOAL_HPP
