#!/usr/bin/env python3

# This file is part of 'sawyer_vr_teleop' 
# Adapted from sawyer_velctrlsim package (https://github.com/michaeltobia/sawyer_velctrlsim/blob/master/src/sim_vel_ctrl.py)

import numpy as np

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool

from intera_core_msgs.msg import JointCommand

import modern_robotics as mr
import sawyer_MR_description as s_des

#### Intera Joint Control Modes (from Intera API documentation) ####
# Must be type int() NOT type Int32() or other std_msgs types
# Intera JointCommand messages are picky
POSITION_MODE = int(1)
VELOCITY_MODE = int(2)
TORQUE_MODE = int(3)
TRAJECTORY_MODE = int(4)

"""
    Node that sets the desired trajectories (manual) and computes the task space velocities for a simulated Sawyer robot
"""

# Simulated Sawyer's Velocity Control class
class SimVelCtrl:
    # Function to initialise velocity control attributes
    def __init__(self):
        # Initialise node
        rospy.init_node("sim_vel_ctrl")

        # Initialise tflistener
        self.tf_lis = tf.TransformListener()

        # Grip Button State Subscriber
        rospy.Subscriber(
            "/oculus_quest2/right/grip_state", Bool, self.right_grip_callback
        )

        # Robot Trajectory Subscriber (generated in traj_gen.py)
        rospy.Subscriber(
            "/desired_trajectory", TransformStamped, self.ctrl_r
        )

        # Joint States Subscriber (obtain the current joint states for the vel_ctrl_sim_interface.py)
        rospy.Subscriber(
            "/joint_states", JointState, self.js_callback
        )

        # Velocity Control Message Publisher
        self.pub = rospy.Publisher(
            "/robot/limb/right/joint_command", JointCommand, queue_size=10
        )

        # Initialise the state of the grip and trigger button
        self.right_grip = False

        ## Controller Gains (PI controller)
        self.Kp = 2.0 * np.eye(6)  # NOTE: Adjust this to reduce in the aggression of trajectory generation
        self.Ki = 0.0 * np.eye(6) 

        ## Robot Description
        self.B_list = s_des.Blist
        self.M = s_des.M

        # Set initial joint states to 0.0
        self.cur_config = np.zeros(10)

        """
        JointCommand message definition:
        int32 mode             # Mode in which to command arm

        string[]  names        # Joint names order for command

        # Fields of commands indexed according to the Joint names vector.
        # Command fields required for a desired mode are listed in the comments
        float64[] position     # (radians)       Required for POSITION_MODE and TRAJECTORY_MODE
        float64[] velocity     # (radians/sec)   Required for VELOCITY_MODE and TRAJECTORY_MODE
        float64[] acceleration # (radians/sec^2) Required for TRAJECTORY_MODE
        float64[] effort       # (newton-meters) Required for TORQUE_MODE

        # Modes available to command arm
        int32 POSITION_MODE=1
        int32 VELOCITY_MODE=2
        int32 TORQUE_MODE=3
        int32 TRAJECTORY_MODE=4
        """

        # Initialise the JointCommand Message from Intera's SDK
        self.joint_ctrl_msg = JointCommand()

        # Declare Sawyer's joints with the parallel gripper (based on the order returned from /joint_states)
        # To check the order of the joints run a 'rostopic echo /joint_states', and assign the order displayed to the JointCommand().names argument
        self.joint_ctrl_msg.names = [
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

        # Set the control mode to the VELOCITY
        self.joint_ctrl_msg.mode = VELOCITY_MODE

        # Set the initial velocity at all joints to 0.0
        self.joint_ctrl_msg.velocity = np.ndarray.tolist(
            np.zeros(len(self.joint_ctrl_msg.names))
        )

        self.it_count = 0  # Iteration count for debug
        self.int_err = np.zeros(6)  # Integrated error 0 -> t

    # Extract the current joint states of Sawyer
    def js_callback(self, js):
        """
        JointState message definition:
        std_msgs/Header header
            uint32 seq
            time stamp
            string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort
        """
        # Stores most recent joint state from /joint_states topic
        self.cur_config = js.position

    # Extract the state of the right controller's grip button
    def right_grip_callback(self, msg):
        self.right_grip = msg.data

    # Extract the desired trajectory and compute the velocity commands for Sawyer's joints
    def ctrl_r(self, X_d):
        #### Filter the joint configuration to accomodate for the only the joints that will have velocity commands send to them ####
        # Remove the 'head_pan', 'right_gripper_l_finger_joint', and 'right_gripper_l_finger_joint' joints since they don't vary (i.e., act as ghost joints)
        cur_theta_list = np.ndarray.tolist(np.delete(self.cur_config, [1, 8, 9]))

        # Get the desired end-effector transform
        # Initialise the End-Effector frame in displacement-quaternion form...
        p_d, Q_d = mr.TFtoMatrix(X_d)

        # ...for ease of use with the Modern Robotics text, the frame is
        # changed to transform matrix form using the tf_listener library
        X_d = self.tf_lis.fromTranslationRotation(p_d, Q_d)

        # Current End effector transform is found using forward kinematics
        X_cur = mr.FKinBody(self.M, self.B_list, cur_theta_list)

        # End Effector transform error X_e is found from [X_e] = log[X^(-1) . X_d]
        X_e = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X_cur), X_d)))

        # Find integral error
        # Can induce instability, use with caution!
        self.int_err = (self.int_err + X_e) * (0.5 / 20)

        # Declare the integration error limit
        limit_err = 0.5

        # Cap the int_err to its limit to prevent runaway (from the root-locus gain)
        for i in range(len(self.int_err)):
            err_i = self.int_err[i]
            if abs(err_i) > limit_err:
                self.int_err[i] = np.sign(err_i) * limit_err

        # Calculate body twist (velocity) command from V_b = K_p . X_e + K_i . int(X_e)
        V_b = np.dot(self.Kp, X_e) + np.dot(self.Ki, self.int_err)

        # Calculate body Jacobian from robot_des and current joint positions
        J_b = mr.JacobianBody(self.B_list, cur_theta_list)

        # Compute the Pseudo-inverse of the Jacobian
        J_b_pinv = np.linalg.pinv(J_b)

        # Calculate Joint velocity commands [theta_dot = pinv(J_b) * V_b]
        theta_dot = np.dot(J_b_pinv, V_b)

        # Declare the joint limit speed
        limit_speed = 0.4

        # Limit joint speed to 0.4 rad/sec (NOTE: Velocity limits are surprisingly high, so be aware when setting them for the desired application)
        for i in range(len(theta_dot)):
            if abs(theta_dot[i]) > limit_speed:
                theta_dot[i] = np.sign(theta_dot[i]) * limit_speed
        rospy.loginfo("Joint vel: %s", theta_dot)

        # Reinsert the head_pan joint velocity at 0 rad/s since it was previously removed and not accounted for in the joint command calculation
        theta_dot = np.insert(theta_dot, 1, 0)
        theta_dot = np.insert(theta_dot, len(theta_dot), 0)
        theta_dot = np.insert(theta_dot, len(theta_dot), 0)

        # Convert to list containers
        self.joint_ctrl_msg.velocity = np.ndarray.tolist(theta_dot)

        # Publish the JointCommand message
        self.pub_joint_ctrl_msg()

    # Publishing velocity commands as a JointCommand message
    def pub_joint_ctrl_msg(self):
        # Add a header time stamp to the JointCommand message
        self.joint_ctrl_msg.header.stamp = rospy.Time.now()

        # Publish the joint velocities if the grip button is pressed
        self.pub.publish(self.joint_ctrl_msg)


def main():
    try:
        # Initialise the velocity controller object
        out = SimVelCtrl()

        # Publish the joint velocities to get the robot running
        out.pub_joint_ctrl_msg()

        rospy.spin()
    except rospy.ROSInterruptException:
        raise e


if __name__ == "__main__":
    main()
