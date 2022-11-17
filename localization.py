import gtsam
import numpy as np
from gtsam import symbol_shorthand
from blockbot import BlockBot, LANDMARK_TAGS
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
    bot.base.reset_odom()

    L = symbol_shorthand.L
    X = symbol_shorthand.X

    bot.camera.tilt(0)

    # Create noise models
    PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.01, 0.01, 0.01], dtype=float))
    ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.01, 0.01, 0.01], dtype=float))
    LANDMARK_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
        np.array([0.1, 0.2], dtype=float))

    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    graph.add(gtsam.PriorFactorPose2(X(0), gtsam.Pose2(0, 0, 0), PRIOR_NOISE))
    initial_estimate.insert(X(0), gtsam.Pose2(0, 0, 0))

    result = None
    marginals = None
    # Move in segments
    odom_history = [[0, 0, 0]]
    seen_landmarks = set()
    for i in range(6):
        time_idx = int(i + 1)
        bot.base.move(.1, 0, 4)

        # Process odometry
        odom = bot.base.get_odom()

        prev_odom = odom_history[-1]
        odom_history.append(odom)

        d_odom = np.subtract(odom, prev_odom)

        odom_pose = gtsam.Pose2(d_odom[0], d_odom[1], d_odom[2])

        graph.add(gtsam.BetweenFactorPose2(X(time_idx - 1), X(time_idx),
                  odom_pose, ODOMETRY_NOISE))
        initial_estimate.insert(
            X(time_idx), gtsam.Pose2(odom[0], odom[1], odom[2]))

        # Process any visible landmarks
        landmarks = [
            tag for tag in bot.tags_data if tag.id[0] in LANDMARK_TAGS]
        for l in landmarks:
            l_id = l.id[0]
            tag = l.pose.pose.pose.position
            # Assume camera is parallel to ground (tilt = 0)
            # Project the tag position to the camera center
            l_range = math.sqrt(tag.x**2 + tag.y**2 + tag.z**2)
            l_bearing = math.atan2(math.sqrt(tag.x**2 + tag.y**2), tag.z)
            graph.add(gtsam.BearingRangeFactor2D(X(time_idx), L(l_id),
                      gtsam.Rot2(l_bearing), l_range, LANDMARK_NOISE))

            # Add estimate for landmark if not seen before
            if l_id not in seen_landmarks:
                # TODO: Better estimate based on odom, bearing, and range
                rng = default_rng()
                p = rng.normal(loc=0, scale=100, size=(2,))
                initial_estimate.insert(L(l_id), p)
                # initial_estimate.insert(L(l_id), gtsam.Pose2(0, 0, 0))

                seen_landmarks.add(l_id)

        # Parameters for the optimization
        parameters = gtsam.GaussNewtonParams()
        parameters.setRelativeErrorTol(1e-5)
        parameters.setMaxIterations(1000)
        optimizer = gtsam.GaussNewtonOptimizer(
            graph, initial_estimate, parameters)
        result = optimizer.optimize()
        marginals = gtsam.Marginals(graph, result)

        # Update estimated pose
        bot.estimated_pose = result.atPose2(X(time_idx))

        print("Estimated pose: ")
        print(bot.estimated_pose)
        print("Covariance:")
        print(marginals.marginalCovariance(X(i+1)))
        print("Odometry measurement")
        print(odom)
        # rospy.sleep(2)

    # Printing and Plotting
    plt.figure(figsize=(8, 8))
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
    
if __name__ == "__main__":
    start_localiztion_demo()
