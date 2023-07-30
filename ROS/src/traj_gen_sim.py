#!/usr/bin/env python3

# This file is part of 'sawyer_vr_teleop' 
# Adapted from sawyer_velctrlsim package (https://github.com/michaeltobia/sawyer_velctrlsim/blob/master/src/traj_gen.py)

import numpy as np
from math import sin, cos, pi

import rospy
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Pose
from visualization_msgs.msg import MarkerArray

import viz_markers as vz
from viz_markers import MarkerType as mk

"""
    Node that generates the goal trajectory the simulated robot must follow
"""


# Trajectory Generator for the robot's end-effector
class TrajectoryGeneratorSim:
    # Initialise movement node functionality
    def __init__(self):
        # Initialise node
        rospy.init_node("ref_trajectory_sim")

        # Initialise desired_trajectory publisher (to set the desired trajectory)
        self.pub_motion = rospy.Publisher(
            "/desired_trajectory", TransformStamped, queue_size=10
        )

        # Initialise Marker Array publisher
        self.marker_arr = rospy.Publisher(
            "/visualization_marker", MarkerArray, queue_size=10
        )

        # Initialise desired trajectory frame TransformStamped message
        self.desired_trajectory_frame = TransformStamped()

        # Set child frame to simulated Sawyer's base
        self.desired_trajectory_frame.child_frame_id = "base"

        # Set desired trajectory frame ID
        self.desired_trajectory_frame.header.frame_id = "trajectory"

    # Extract the pose data from the right-hand controller
    def right_callback(self, msg):
        self.pos_x = msg.position.x
        self.pos_y = msg.position.y
        self.pos_z = msg.position.z

        self.ori_w = msg.orientation.w
        self.ori_x = msg.orientation.x
        self.ori_y = msg.orientation.y
        self.ori_z = msg.orientation.z

    # Trajectory setting for the simulated arm
    def trajectory(self, t):
        ## Trajectory is specified relative to base frame (default frame)

        ##### DESIRED TRANSLATION 'p_d -> vel_ctrl' #####

        # X-Position (blue axis)q
        # x_d > 0 --> away from controller tower

        # Y-Position (red axis)
        # y_d > 0 --> left when on control tower side facing Sawyer

        # Z-Position (green axis)
        # z_d > 0 --> up

        self.pos_x = 0.1 * cos(2 * pi * t / 5.0) + 0.6
        self.pos_y = 0.0
        self.pos_z = 0.5

        x_d = self.pos_x
        y_d = self.pos_y
        z_d = self.pos_z

        ##### DESIRED ROTATION 'Q_d -> vel_ctrl' #####

        # Quaternion (w,x,y,z)
        # w - amount of rotation that happens about the axes
        # x,y,z - axis about which a rotation will occur

        theta = pi / 2.0
        self.ori_w = cos(theta / 2.0)
        self.ori_x = 0.0 * sin(theta / 2.0)
        self.ori_y = 1.0 * sin(theta / 2.0)
        self.ori_z = 0.0 * sin(theta / 2.0)

        q0_d = self.ori_w  # Rotation magnitude w (*not quaternion magnitude)
        q1_d = self.ori_x  # Unit axis 'i' / base frame axis x
        q2_d = self.ori_y  # Unit axis 'j' / base frame axis y
        q3_d = self.ori_z  # Unit axis 'k' / base frame axis z

        # Specified desired position
        p_d = [x_d, y_d, z_d]

        # Convert the quaternion to an array and normalise list
        Q_d = np.array([q0_d, q1_d, q2_d, q3_d])  # convert to np.array()
        Q_d = Q_d / np.linalg.norm(Q_d)  # normalize Q_d
        Q_d = np.ndarray.tolist(Q_d)  # convert back to list
        return p_d, Q_d

    # Publish the VR trajectory
    def publish_trajectory(self):
        # Stamp desired trajectory TransformStamped message header
        self.desired_trajectory_frame.header.stamp = rospy.Time.now()

        # Apply desired trajectory to TransformStamped message
        p_d, Q_d = self.trajectory(rospy.get_time())
        self.desired_trajectory_frame.transform.translation = Vector3(
            p_d[0], p_d[1], p_d[2]
        )
        self.desired_trajectory_frame.transform.rotation = Quaternion(
            Q_d[0], Q_d[1], Q_d[2], Q_d[3]
        )

        # Publish TransformStamped() desired trajectory message
        self.pub_motion.publish(self.desired_trajectory_frame)
        rate.sleep()

    # Publish the VR-set goal trajectory markers
    def pub_marker(self):
        ###### SET THE TRAJECTORY (NOTE: Change this depending on whether you want to simulate or map the VR Pose) #######
        p_d, Q_d = self.trajectory(rospy.get_time())

        # Declare the dictionary containing the goal trajectory
        objects = [
            {
                "position": {"x": p_d[0], "y": p_d[1], "z": p_d[2]},
                "orientation": {"x": Q_d[1], "y": Q_d[2], "z": Q_d[3], "w": Q_d[0]},
            },
        ]

        # Create the marker arrays as ARROWS representing the goal trajectories (look at viz_markers.py for the different types of markers available)
        self.mrk_arr = vz.create_marker_array(objects, mk(0))

        # Publish the marker arrays if the controller's grip button is pressed
        self.marker_arr.publish(self.mrk_arr)


if __name__ == "__main__":
    try:
        # Initialise the trajectory generator
        out = TrajectoryGeneratorSim()

        # Global rate 20 Hz for simulator
        rate = rospy.Rate(20)

        # While the program is running publish the trajectory and corresponding markers
        while not rospy.is_shutdown():
            out.publish_trajectory()
            out.pub_marker()
    except rospy.ROSInterruptException:
        raise e
