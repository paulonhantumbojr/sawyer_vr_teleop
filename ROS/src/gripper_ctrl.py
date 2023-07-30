#! /usr/bin/env python3

# Copyright (c) 2013-2018, Rethink Robotics Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
    Node developed based on the Intera Examples' script on the Gripper control with joysticks.
    Ref: https://support.rethinkrobotics.com/support/solutions/articles/80000980355-gripper-example
"""

import argparse

import rospy
from std_msgs.msg import Bool

import intera_interface
from intera_interface import CHECK_VERSION


# Gripper Control
class GripCtrl:
    # Initialise class attributes
    def __init__(self):
        # Initialise node
        rospy.init_node("sdk_gripper_vr")

        # Initialise the state of the binded buttons
        self.primary_btn = False
        self.secondary_btn = False

        # Primary Button State Subscriber
        rospy.Subscriber(
            "/oculus_quest2/primary/button_state", Bool, self.primary_btn_callback
        )

        # Secondary Button State Subscriber
        rospy.Subscriber(
            "/oculus_quest2/secondary/button_state", Bool, self.secondary_btn_callback
        )

    # Extract the state of the primary button
    def primary_btn_callback(self, msg):
        self.primary_btn = msg.data

    # Extract the state of the secondary button
    def secondary_btn_callback(self, msg):
        self.secondary_btn = msg.data

    # Initialise the gripper
    def initialise_vr(self, limb):
        # Initialise interfaces
        rs = intera_interface.RobotEnable(CHECK_VERSION)
        init_state = rs.state()
        gripper = None
        original_deadzone = None

        # Cleaning function (executed on shutdown)
        def clean_shutdown():
            if gripper and original_deadzone:
                gripper.set_dead_zone(original_deadzone)

        try:
            # Instantiate the gripper object
            gripper = intera_interface.Gripper(limb + "_gripper")
        except (ValueError, OSError) as e:
            rospy.logerr("Could not detect an electric gripper attached to the robot.")
            clean_shutdown()
            return
        rospy.on_shutdown(clean_shutdown)

        # Command abbreviations
        openfnct = lambda: not self.primary_btn
        closefnct = lambda: self.primary_btn
        calibfnct = lambda: self.secondary_btn

        # Print the bindings list
        def print_help(bindings_list):
            print("Press Ctrl-C to quit.")
            for bindings in bindings_list:
                for test, _cmd, doc in bindings:
                    if callable(doc):
                        doc = doc()
                    print("%s: %s" % (str(test()), doc))

        # Offsetting the gripper position
        def offset_position(offset_pos):
            cmd_pos = max(
                min(gripper.get_position() + offset_pos, gripper.MAX_POSITION),
                gripper.MIN_POSITION,
            )
            gripper.set_position(cmd_pos)

        # Updating velocity
        def update_velocity(offset_vel):
            cmd_speed = max(
                min(gripper.get_cmd_velocity() + offset_vel, gripper.MAX_VELOCITY),
                gripper.MIN_VELOCITY,
            )
            gripper.set_cmd_velocity(cmd_speed)

        original_deadzone = gripper.get_dead_zone()
        # Possible deadzone values: 0.001 - 0.002
        gripper.set_dead_zone(0.001)
        num_steps = 8.0
        percent_delta = 1.0 / num_steps
        velocity_increment = (
            gripper.MAX_VELOCITY - gripper.MIN_VELOCITY
        ) * percent_delta
        position_increment = (
            gripper.MAX_POSITION - gripper.MIN_POSITION
        ) * percent_delta

        # Assign the functionalities of the gripper to the controller bindings
        bindings_list = []
        bindings = [
            # Key: (test, command, description)
            (openfnct, (gripper.open, []), "open"),
            (closefnct, (gripper.close, []), "close"),
            (calibfnct, (gripper.calibrate, []), "calibrate"),
        ]
        bindings_list.append(bindings)

        # Enable the robot
        rospy.loginfo("Enabling robot...")
        rs.enable()
        rate = rospy.Rate(100)
        print_help(bindings_list)   

        # While the node is still running map the control of the gripper to the controller inputs
        while not rospy.is_shutdown():
            # Test each controller condition and call binding cmd if true
            for test, cmd, doc in bindings:
                if test():
                    print(doc)
                    cmd[0](*cmd[1])
            rate.sleep()

        # Force shutdown call if caught by key handler
        rospy.signal_shutdown("Example finished.")


def main():
    """
    RSDK Gripper Example: Oculus Control
    Use your Oculus Quest 2 VR controller's buttons to control and configure grippers.
    """
    epilog = """
        See help inside within the example for the key bindings.
    """
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(
            ("Cannot detect any limb parameters on this robot. " "Exiting."), "ERROR"
        )
        return
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(
        formatter_class=arg_fmt, description=main.__doc__, epilog=epilog
    )
    parser.add_argument(
        "-l",
        "--limb",
        dest="limb",
        default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the gripper oculus control",
    )
    args = parser.parse_args(rospy.myargv()[1:])

    # Execute the gripper control functionalities
    try:
        # Declare the Gripper object
        grip_ctrl = GripCtrl()

        # Initialise and map the VR controller to the gripper functionalities
        grip_ctrl.initialise_vr(args.limb)

    except rospy.ROSInterruptException:
        raise e


if __name__ == "__main__":
    main()
