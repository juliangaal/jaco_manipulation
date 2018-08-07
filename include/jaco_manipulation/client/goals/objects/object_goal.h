//
// Created by chitt on 8/6/18.
//

#ifndef PROJECT_OBJECTGOAL_HPP
#define PROJECT_OBJECTGOAL_HPP
#include "jaco_manipulation/client/goals/pose_goal.h"

namespace jaco_manipulation {
namespace client {
namespace goals {
namespace object_helper {

/**
 * Object to grasp, defined by bounding box
 */
struct Object {
  std::string description;
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
  explicit ObjectGoal(const object_helper::LimitedPose &grasp_pose_goal, const std::string &description = "grasp goal");


  explicit ObjectGoal(const object_helper::Object &object_goal, const std::string &description = "grasp box goal");

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
   * @param grasp_helper::Object object defined by bounding box
   */
  void adjustPoseToCenterOfObject(const object_helper::Object &object);

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
};

} // namespace objects
} // namespace goals
} // namespace client
} // namespace jaco_manipulation

#endif //PROJECT_OBJECTGOAL_HPP
