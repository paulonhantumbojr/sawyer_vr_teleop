#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker, MarkerArray

from enum import Enum


"""
    Custom class that creates marker arrays of a selected type for objects to be placed within the simulated environment
    For the various marker types refer to the wiki: http://wiki.ros.org/rviz/DisplayTypes/Marker
"""


# Marker type class
class MarkerType(Enum):
    ARROW = 0
    CUBE = 1
    SPHERE = 2
    CYLINDER = 3


# Marker counters
class Counter:
    def __init__(self):
        self.cnt = 0


# Declare a counter object
counter = Counter()


# Functionality to generate a marker array
def create_marker_array(objects, type_):
    # Declare a marker array object
    marker_array = MarkerArray()

    #### Go through each object and set the desired features of the markers ####
    for object in objects:
        # Marker placeholder
        marker = Marker()

        # Header information
        marker.header.frame_id = "base"
        marker.header.stamp = rospy.Time.now()

        # Namespace and counter sequence
        marker.ns = "goal"
        marker.id = counter.cnt
        counter.cnt += 1

        # Marker lifetime
        marker.lifetime = rospy.Duration(0.1)

        # Marker dimensions
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Marker color
        marker.color.r = 1.0
        marker.color.g = 177.0 / 255.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        # Marker shape
        if type_ == MarkerType(0):
            marker.type = Marker.ARROW
        elif type_ == MarkerType(1):
            marker.type = Marker.CUBE
        elif type_ == MarkerType(2):
            marker.type = Marker.SPHERE
        elif type_ == MarkerType(3):
            marker.type = Marker.CYLINDER

        # ADD the markers to the scene
        marker.action = Marker.ADD

        # Marker position
        marker.pose.position.x = object["position"]["x"]
        marker.pose.position.y = object["position"]["y"]
        marker.pose.position.z = object["position"]["z"]

        # Marker orientation
        marker.pose.orientation.x = object["orientation"]["x"]
        marker.pose.orientation.y = object["orientation"]["y"]
        marker.pose.orientation.z = object["orientation"]["z"]
        marker.pose.orientation.w = object["orientation"]["w"]

        # Add the marker to the array
        marker_array.markers.append(marker)

    return marker_array
