#!/usr/bin/env python3

# This file is part of 'sawyer_vr_teleop' 
# Adapted from sawyer_velctrlsim package (https://github.com/michaeltobia/sawyer_velctrlsim/blob/master/src/vel_ctrl_sim_interface.py)

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import JointCommand

"""
    Node that receives the joint velocites computed from the task space velocity computations and assigns them to the corresponding /joint_states of the simulated robot model 
    These velocities are also used to compute the position from the vertical axis of each joint (in rad) 
"""


# Simulation Interface class
class SimInterface:
    def __init__(self):
        # Initialise node
        rospy.init_node("vel_ctrl_sim_interface")

        # Initialise /joint_states puslisher for the simulated robot
        self.pub = rospy.Publisher(
            "/joint_states", JointState, queue_size=10
        )

        # Initialise joint_command subscriber for JointCommand control messages (from sim_vel_ctrl.py)
        rospy.Subscriber(
            "/robot/limb/right/joint_command", JointCommand, self.commandToState
        )

        # Initialise the JointState() Message from ROS sensors' library
        self.joint_state = JointState()

        # Declare Sawyer's joints with the parallel gripper in order from the /joint_states message
        # Run a 'rostopic echo /joint_states' to confirm the order of the joint names
        self.joint_state.name = [
            "right_j0",
            "head_pan",
            "right_j1",
            "right_j2",
            "right_j3",
            "right_j4",
            "right_j5",
            "right_j6",
            "right_gripper_l_finger_joint",
            "right_gripper_r_finger_joint",
        ]

        # Set the initial joint positions and velocities to 0 (number of joints must correspond to the /joint_states)
        self.joint_state.position = np.ndarray.tolist(np.zeros(10))
        self.joint_state.velocity = np.ndarray.tolist(np.zeros(10))

    # Extract the joint velocity commands and compute the joint state positions
    def commandToState(self, vel_command):
        vel_command = np.asarray(vel_command.velocity)  # convert to np.array()
        pos_command = vel_command * (1.0 / 20)  # velocity/rate = position
        pos_command[1] = 0  # Assign a 0 head rotation for the /head_pan joint
        pos_command[8] = 0  # Assign a 0 finger translation for the lfinger joint
        pos_command[9] = 0  # Assign a 0 finger translation for the rfinger joint
        self.joint_state.position += pos_command  # Increment position
        self.joint_state.velocity = vel_command  # Assign the velocity commands
        self.joint_state.header.stamp = rospy.Time.now()  # Stamp joint positions
        self.pub.publish(self.joint_state)  # Publish the Joint state messages


if __name__ == "__main__":
    try:
        # Initialise the simulation interface
        out = SimInterface()
        rospy.spin()
    except rospy.ROSInterruptException:
        raise e
