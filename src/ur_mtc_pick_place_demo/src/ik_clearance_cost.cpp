from moveit_commander.robot_state import RobotState
from moveit_commander.robot_model import RobotModel
from moveit_commander.robot_model_loader import RobotModelLoader
from geometry_msgs.msg import Pose

# Load robot model
robot_model_loader = RobotModelLoader("robot_description")
robot_model = robot_model_loader.getRobotModel()
robot_state = RobotState(robot_model)

# Get joint model group
joint_model_group = robot_model.get_joint_model_group("manipulator")  # use your MoveIt group name

# Set desired pose
target_pose = Pose()
target_pose.position.x = 0.4
target_pose.position.y = 0.1
target_pose.position.z = 0.2
target_pose.orientation.w = 1.0

# Compute IK
found_ik = robot_state.setFromIK(joint_model_group, target_pose)

if found_ik:
    joint_values = robot_state.getJointGroupPositions(joint_model_group)
    print("IK Solution:", joint_values)
else:
    print("IK failed")
