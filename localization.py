import gtsam
import numpy as np
from gtsam import symbol_shorthand
from blockbot import BlockBot, LANDMARK_TAGS, RobotActionState
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import matplotlib.pyplot as plt
from matplotlib import patches
import math
from numpy.random import default_rng


def plot_point2_on_axes(axes, pose, P=None, color=None):
    p_x = pose.x()
    p_y = pose.y()
    if color is not None:
        axes.plot([p_x], [p_y], marker='.', color=color, markersize=8)
    else:
        axes.plot([p_x], [p_y], marker='.', markersize=8)
    if P is not None:
        w, v = np.linalg.eigh(P)
        # this corresponds to 95%
        k = 2.447746830681

        angle = np.arctan2(v[1, 0], v[0, 0])
        e1 = patches.Ellipse([p_x, p_y],
                             np.sqrt(w[0]) * 2 * k,
                             np.sqrt(w[1]) * 2 * k,
                             np.rad2deg(angle),
                             fill=False,
                             alpha=0.3)
        axes.add_patch(e1)


def plot_result_point2(result, marginals):
    ax = plt.gca()
    for key in result.keys():
        plot_point2_on_axes(
            ax, result.atPose2(key), marginals.marginalCovariance(key))

    plt.axis('equal')
    plt.savefig('localization_prob.png')


def print_result_point2(result, marginals):
    for key in result.keys():
        print(f"Point[{key}]:\n"
              f"  Location: {result.atPoint2(key)}, \n"
              f"  Covariance: \n{marginals.marginalCovariance(key)}\n")


def start_localiztion_demo():
    bot = BlockBot()
    bot.camera.tilt(0)

    bot.action_state = RobotActionState.MOVE_FORWARD

    # Move in segments
    for i in range(6):
        bot.base.move(.1, 0, 4)
        bot.update_position_estimate()

    # Printing and Plotting
    # plt.figure(figsize=(8, 8))
    # plt.plot(x_true, y_true, ':m', alpha=0.3)
    # plt.plot(landmarks[:, 0], landmarks[:, 1], 'mo', alpha=0.2)
    # print_result_point2(result)

    # plot_result_point2(result, marginals)

    # # Add the observations for the landmarks
    # for i, landmark in enumerate(landmarks):
    #     for det in landmark.detections:
    #         (l_dx, l_dy), pose, pose_id = det
    #         graph.add(gtsam.BetweenFactorPoint2(X(int(pose_id)), L(i), gtsam.Point2(l_dx, l_dy), LANDMARK_NOISE))

    # Set the initial position of the landmarks
    # Note: you can use the positions already stored in each landmark as an initial guess
    # l_x, l_y = landmark.position
    # initial_estimate.insert(L(i), gtsam.Point2(l_x, l_y))

    # Get the values of the landmark positions.
    # Note: you don't need to do it like this, but this is how I've done it.
    # I recommend that you use the "symbols" L and X; it makes bookkeeping easier.
    # for lind, l in enumerate(landmarks):
    #     l.position = result.atPoint2(L(lind))

    # return result.atPoint2(X(len(dxs))), result, marginals
    bot.action_state = RobotActionState.WAIT


if __name__ == "__main__":
    start_localiztion_demo()
