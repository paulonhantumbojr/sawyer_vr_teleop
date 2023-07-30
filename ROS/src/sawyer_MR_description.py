#! /usr/bin/env python3

# This file is part of 'sawyer_vr_teleop'
# Script obtained from sawyer_velctrlsim package (https://github.com/michaeltobia/sawyer_velctrlsim/blob/master/src/sawyer_MR_description.py)

import numpy as np
from math import cos, sin, radians

s10 = sin(radians(10))
c10 = cos(radians(10))

# List of the joint screw axes in the end-effector frame
Blist = np.array(
    [
        [s10, -c10, 0.0, -1.0155 * c10, -1.0155 * s10, -0.1603],
        [-c10, -s10, 0.0, -0.9345 * s10, 0.9345 * c10, 0.0],
        [0.0, 0.0, 1.0, -0.0322 * s10, 0.0322 * c10, 0.0],
        [-c10, -s10, 0.0, -0.5345 * s10, 0.5345 * c10, 0.0],
        [0.0, 0.0, 1.0, 0.1363 * s10, -0.1363 * c10, 0.0],
        [-c10, -s10, 0.0, -0.1345 * s10, 0.1345 * c10, 0.0],
        [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    ]
)

Blist = Blist.T

# Homogeneous transformation matrix of the position and orientation of the robot's end-effector
# First 3x3 matrix represents the rotation operator [R = Rot(ῶ,θ)] and the 3x1 matrix represents the translation operator [Trans(p)]
M = np.array(
    [
        [0.0, 0.0, 1.0, 1.0155],
        [-c10, -s10, 0.0, 0.1603],
        [s10, -c10, 0.0, 0.317],
        [0.0, 0.0, 0.0, 1.0],
    ]
)
