import math
import gtsam
import numpy as np
from gtsam import symbol_shorthand

L = symbol_shorthand.L
X = symbol_shorthand.X

# Create noise models
PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([0.01, 0.01, 0.01], dtype=float))
ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([0.01, 0.01, math.pi/32], dtype=float))
LANDMARK_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([0.5, 0.5], dtype=float))


def calc_pos_from_bearing_range(pose, l_bearing, l_range):
    '''Calculate the global coordinate of landmark based on 
    current pose, bearing, and range'''
    # Calculate relative x and y from robot pose
    dx = l_range * np.cos(l_bearing)
    dy = l_range * np.sin(l_bearing)

    # Rotate opposite robot pose to get global change in x and y from pose
    # Add to robot's pose coordinates
    r_x, r_y, r_theta = pose
    l_x = r_x + (dx * np.cos(-r_theta) + dy * np.sin(-r_theta))
    l_y = r_y + (dx * np.sin(r_theta) + dy * np.cos((-r_theta)))

    return (l_x, l_y)


def calc_bearing_range_from_tag(tag, camera_tilt):
    '''Takes AprilTag detection data and calculates a bearing and range,
    taking into account the tilt of the camera.'''
    # Tag data (in meters)
    # z = distance to center of tag to center of camera
    # x = real horizontal distance from camera center, left relative to robot is postive
    # y = real vertical distance from camera center, down is positive

    # Project the tag distance to the camera center parallel to the ground based on camera tilt
    dist_to_camera_center = math.sqrt(tag.z**2 - tag.x**2 - tag.y**2) * np.cos(camera_tilt)

    # Adjust for placement of camera from center of LoCoBot
    camera_x_dist = 0.07
    t_range = math.sqrt(tag.x**2 + (dist_to_camera_center + camera_x_dist)**2)

    # Find the bearing angle from LoCobot center to tag
    t_bearing = math.asin(tag.x / t_range)

    # Flip bearing for correct bot rotation direction

    return -t_bearing, t_range


# BlockbotLocalizer is now in it's own catkin package.
# TODO: Move the above functions somewhere else?