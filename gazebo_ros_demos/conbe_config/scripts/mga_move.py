import rospy

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import moveit_commander
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, MoveItErrorCodes
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume

import actionlib

from shape_msgs.msg import SolidPrimitive

import numpy as np

rospy.init_node('move_group_client')
action_topic = '/move_group'
client = actionlib.SimpleActionClient(action_topic, MoveGroupAction)

move_group = 'conbe'
group = moveit_commander.MoveGroupCommander(move_group)

planning_frame = group.get_planning_frame()
print('planning_frame:', planning_frame)
# ('planning_frame:', '/panda_link0')

eef_link = group.get_end_effector_link()
print('eef_link:', eef_link)
# ('eef_link:', 'panda_link8')

tolerance = 0.01

pose_goal = Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 1.0
pose_goal.position.y = 0.15
pose_goal.position.z = 0.4 

action_goal = MoveGroupGoal()
action_goal.request.group_name = move_group

goal_constraint = Constraints()
goal_constraint.position_constraints.append(PositionConstraint())
goal_constraint.position_constraints[0].header.frame_id = planning_frame
goal_constraint.position_constraints[0].link_name = eef_link

solid_primitive = SolidPrimitive()
solid_primitive.dimensions = [tolerance * tolerance]
solid_primitive.type = solid_primitive.SPHERE

bounding_volume = BoundingVolume()
bounding_volume.primitives.append(solid_primitive)
bounding_volume.primitive_poses.append(pose_goal)
goal_constraint.position_constraints[0].constraint_region = bounding_volume

goal_constraint.orientation_constraints.append(OrientationConstraint())
goal_constraint.orientation_constraints[0].header.frame_id = planning_frame
goal_constraint.orientation_constraints[0].link_name = eef_link
goal_constraint.orientation_constraints[0].orientation = pose_goal.orientation
goal_constraint.orientation_constraints[0].absolute_x_axis_tolerance = tolerance
goal_constraint.orientation_constraints[0].absolute_y_axis_tolerance = tolerance
goal_constraint.orientation_constraints[0].absolute_z_axis_tolerance = tolerance

action_goal.request.goal_constraints.append(goal_constraint)
action_goal.request.num_planning_attempts = 5
action_goal.request.allowed_planning_time = 5
action_goal.planning_options.replan = True

action_goal.planning_options.plan_only = False


# joint_goal = np.array([0,0,0,0,0,0])

# action_goal.set_joint_value_target(joint_goal)
# action_goal.set_position_target([x, y, z], self.end_effector_link)
# action_goal.set_pose_target([x, y, z, roll, pitch, yaw], self.end_effector_link)

client.send_goal(action_goal)

rospy.sleep(3)