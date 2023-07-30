#!/usr/bin/env python3

# This file is part of 'sawyer_vr_teleop'
# Adapted from sawyer_velctrlsim package (https://github.com/michaeltobia/sawyer_velctrlsim/blob/master/src/sawyer_vel_ctrl.py)

import numpy as np

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool

import intera_interface
from intera_core_msgs.msg import JointCommand

import modern_robotics as mr
import sawyer_MR_description as s_des

# Intera Joint Control Modes (Reference: https://sdk.rethinkrobotics.com/intera/Arm_Control_Systems#Joint_Control_Modes:~:text=type%20std_msgs/Float64-,MESSAGE%20DEFINITION,-The%20JointCommand%20message)
# Must be type int() NOT type Int32() or other std_msgs types
# Intera JointCommand messages are picky
POSITION_MODE = int(1)
VELOCITY_MODE = int(2)
TORQUE_MODE = int(3)
TRAJECTORY_MODE = int(4)

"""
    Node that sets the desired trajectories based on the pose from the VR headset controllers and computes the task space velocities for a Sawyer robot control
"""


# Sawyer Velocity Control
class VelCtrl:
    # Initialise class attributes
    def __init__(self):
        # Initialise node
        rospy.init_node("sawyer_vel_ctrl")

        # Initialise TransformListener package
        self.tf_lis = tf.TransformListener()

        # Grip Button State Subscriber
        rospy.Subscriber(
            "/oculus_quest2/right/grip_state", Bool, self.right_grip_callback
        )

        # Robot Trajectory Subscriber (generated in traj_gen.py)
        rospy.Subscriber(
            "/desired_trajectory", TransformStamped, self.ctrl_r
        )

        # Joint States Subscriber (obtain the current joint states for the vel_ctrl_sim_interface)
        rospy.Subscriber(
            "/robot/joint_states", JointState, self.js_callback
        )

        # Velocity Control Message Publisher
        self.pub = rospy.Publisher(
            "/robot/limb/right/joint_command", JointCommand, queue_size=10
        )

        # Initialise the state of the grip button
        self.right_grip = False

        """
            Initialise the Sawyer's head display object
            Ref: https://rethinkrobotics.github.io/intera_sdk_docs/5.1.0/intera_interface/html/index.html
        """
        self.head_display = intera_interface.HeadDisplay()

        ## Controller Gain Parameters (PI Controller)
        self.Kp = 2.0 * np.eye(6)  # NOTE: Adjust this to reduce in the aggression of trajectory generation
        self.Ki = 0.0 * np.eye(6)

        ## Robot Description (refer to sawyer_MR_description.py)
        self.B_list = s_des.Blist
        self.M = s_des.M

        # Set initial joint states to 0, may need to change in IRL use
        self.cur_config = np.zeros(9)

        # Wait for actual joint_states to be stored by js_store() callback (NOTE: Don't change the 'is' to '==')
        while np.sum(self.cur_config) is 0:
            # If the current joint configurations of the robot are set to 0 put the thread to sleep (similar to a rate_limiter.sleep())
            rospy.sleep(0.1)

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
        """

        # Joint Command Message Initialisation
        self.joint_ctrl_msg = JointCommand()

        # To check the order of the joints run a 'rostopic echo /robot/joint_states', and assign the order displayed to the JointCommand().names argument
        self.joint_ctrl_msg.names = [
            "head_pan",
            "right_j0",
            "right_j1",
            "right_j2",
            "right_j3",
            "right_j4",
            "right_j5",
            "right_j6",
            "torso_t0",
        ]

        # Set the control mode to the VELOCITY
        self.joint_ctrl_msg.mode = VELOCITY_MODE

        # Set the initial velocity at all joints to 0.0
        self.joint_ctrl_msg.velocity = np.ndarray.tolist(
            np.zeros(len(self.joint_ctrl_msg.names))
        )

        self.it_count = 0  # Iteration count for debug
        self.int_err = np.zeros(6)  # Integrated error 0 -> t

    # Extract the state of the right controller's grip button
    def right_grip_callback(self, msg):
        self.right_grip = msg.data

    # Extract the desired trajectory and compute the velocity commands for Sawyer's joints
    def ctrl_r(self, X_d):
        #### Filter the configuration of the joints that will be sent velocity commands ####
        # Remove the first and last elements (in that order) of the converted list,
        # since the /robot/joint_states for the 'head_pan' and 'torso_t0' don't vary (i.e., act as ghost joints)
        cur_theta_list = np.ndarray.tolist(np.delete(self.cur_config, [0, -1]))

        # Get the desired end-effector transform
        # Initialise the End-Effector frame in displacement-quaternion form...
        p_d, Q_d = mr.TFtoMatrix(X_d)

        # ...for ease of use with the Modern Robotics text, the frame is
        # changed to transform matrix form using the tflistener library
        X_d = self.tf_lis.fromTranslationRotation(p_d, Q_d)

        # Current end-effector transform is found using forward kinematics
        X_cur = mr.FKinBody(self.M, self.B_list, cur_theta_list)

        # End-Effector transform error X_e is found from [X_e] = log(X^(-1) * X_d)
        X_e = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(X_cur), X_d)))

        # Find integral error
        ## Can induce instability, use with caution!
        self.int_err = (self.int_err + X_e) * (0.5 / 20)

        # Declare the integration error limit
        limit_err = 0.5

        # Hard limit each integration error element to 0.5 to prevent runaway
        for i in range(len(self.int_err)):
            err_i = self.int_err[i]
            if abs(err_i) > limit_err:
                self.int_err[i] = np.sign(err_i) * limit_err

        # Calculate body twist command from V_b = K_p . X_e + K_i . int(X_e)
        V_b = np.dot(self.Kp, X_e) + np.dot(self.Ki, self.int_err)

        # Calculate body Jacobian from robot_des and current joint positions
        J_b = mr.JacobianBody(self.B_list, cur_theta_list)

        # Compute the Pseudo-inverse of the Jacobian
        J_b_pinv = np.linalg.pinv(J_b)

        # Calculate joint velocity command [theta_dot = pinv(J_b) . V_b]
        theta_dot = np.dot(J_b_pinv, V_b)

        # Declare the joint's limit speed (NOTE: For safety purposes, set this to a value below or equal to 0.6 rad/s. Speed range spans from 0.0-1.0 rad/s)
        limit_speed = 0.4

        # Limit joint speed to 0.4 rad/sec (NOTE: Velocity limits are surprisingly high)
        for i in range(len(theta_dot)):
            if abs(theta_dot[i]) > limit_speed:
                theta_dot[i] = np.sign(theta_dot[i]) * limit_speed
        rospy.loginfo("Joint vel: %s", theta_dot)

        # Reinsert the 'head_pan' and 'torso_j0' joint velocities at 0.0 rad/s since they were previously removed and not accounted for in the joint command calculation
        theta_dot = np.insert(theta_dot, 0, 0)
        theta_dot = np.insert(theta_dot, len(theta_dot), 0)

        # Convert JointCommand messages to a list
        self.joint_ctrl_msg.velocity = np.ndarray.tolist(theta_dot)

        # Publish the JointCommand message (called as the function defined below)
        self.pub_joint_ctrl_msg()

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
        # Stores most recent joint states from the /robot/joint_states topic
        self.cur_config = js.position

    # Publishing velocity commands as a JointCommand message
    def pub_joint_ctrl_msg(self):
        # Add a header time stamp
        self.joint_ctrl_msg.header.stamp = rospy.Time.now()

        # Publish the joint velocities if the grip button is pressed
        if self.right_grip == True:
            self.pub.publish(self.joint_ctrl_msg)

    # Display images in Sawyer's head display
    def disp_image(self):
        # Declare the list of images for Sawyer's head to loop through
        single_image_path = [
            "/home/nhantastrew/Workspaces/sawyer_ws/src/sawyer_vr_teleop/images/caution_sign.png"
        ]

        # Display the image in the head display
        self.head_display.display_image(single_image_path, False, 1.0)


def main():
    try:
        # Initialise controller
        out = VelCtrl()

        # Publish the joint velocities to get the robot running
        out.pub_joint_ctrl_msg()

        # Display an image on sawyer's display
        out.disp_image()

        rospy.spin()
    except rospy.ROSInterruptException:
        raise e


if __name__ == "__main__":
    main()
