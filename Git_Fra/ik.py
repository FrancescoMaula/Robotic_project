#!/usr/bin/env python3

import rospy
from moveit_msgs.srv import *
from moveit_msgs.msg import RobotState, Constraints, PositionIKRequest
from moveit_commander import RobotCommander, MoveGroupCommander

def print_inverse_kinematics_solution():
    # Initialize ROS node
    rospy.init_node('inverse_kinematics_example')

    # Initialize MoveIt commander
    robot = RobotCommander()
    group = MoveGroupCommander('arm')  # Replace 'manipulator' with your robot's planning group

    # Create the IK request
    ik_request = PositionIKRequest()
    ik_request.group_name = 'arm'  # Replace 'manipulator' with your robot's planning group
    ik_request.robot_state = RobotState()
    ik_request.robot_state.joint_state.name = group.get_active_joints()
    ik_request.robot_state.joint_state.position = [0.0] * len(group.get_active_joints())
    ik_request.avoid_collisions = True
    print("Robot_state: %s",RobotState())


    current_pose = group.get_current_pose().pose
    print("Current: %s",current_pose)
    # Set the desired pose for IK calculation
    pose = group.get_random_pose().pose  # Replace with the desired pose
    print("Pose_goal: %s",pose)
    ik_request.pose_stamped.header.frame_id = 'base_link'  # Replace 'base_link' with your robot's base frame
    ik_request.pose_stamped.pose = pose

    # Set the constraints (if any)
    constraints = Constraints()
    # Add constraints here if needed
    ik_request.constraints = constraints

    # Call the IK service
    rospy.wait_for_service('/compute_ik')
    try:
        compute_ik = rospy.ServiceProxy('/compute_ik', moveit_msgs.srv.GetPositionIK)
        ik_response = compute_ik(ik_request)
        if ik_response.error_code.val == ik_response.error_code.SUCCESS:
            joint_positions = dict(zip(ik_response.solution.joint_state.name,
                                        ik_response.solution.joint_state.position))
            print("Inverse kinematics solution:")
            for joint, position in joint_positions.items():
                print(f"{joint}: {position}")
            group.go(ik_response.solution.joint_state.position, wait=True)
            group.stop()
        else:
            print("Inverse kinematics calculation failed with error code:", ik_response.error_code.val)
    except rospy.ServiceException as e:
        print("Service call failed:", e)

if __name__ == '__main__':
    print_inverse_kinematics_solution()
