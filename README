A fully catkin-ized version of manipulation using doro.

###grasp_pose_generator
===> This creates 4 semi-hardcoded grasp poses
using the position of the object to be grasped. It's a ros server
interface. 

###jaco_manipulation
===> Provides an action server interface (action
messages format defined in action/PlanAndMoveArm.action). The input is
a 3D pose. This is one of the 4 poses obtained from Grasp Pose
Generation or Put Down Points Generation.

###put_down_points_generator
===> This generates points on a table where
an object currently being held can be safely put down. Relies on
cluster_extraction to extract the centroids of the clusters and the
table. 