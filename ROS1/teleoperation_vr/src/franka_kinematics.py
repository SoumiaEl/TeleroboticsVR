#!/usr/bin/env python
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from ikpy.chain import Chain
from ikpy import geometry_utils

# Load the URDF file
chain = Chain.from_urdf_file(".urdf")

def calculate_joint_positions(pose):
    # Convert the ROS pose to a 4x4 transformation matrix
    trans_matrix = geometry_utils.to_transformation_matrix(
        [pose.position.x, pose.position.y, pose.position.z],
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    )

    # Solve the inverse kinematics
    joint_positions = chain.inverse_kinematics(trans_matrix)
    return joint_positions

def callback(data):
    joint_positions = calculate_joint_positions(data.pose)
    msg = JointState()
    msg.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    msg.position = joint_positions
    pub.publish(msg)

rospy.init_node('joint_commander')
rospy.Subscriber("/ee_target_pose", PoseStamped, callback)
pub = rospy.Publisher('/franka/desired_joint_state', JointState, queue_size=10)
rospy.spin()
