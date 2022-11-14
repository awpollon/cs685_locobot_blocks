import gtsam
import numpy as np
from gtsam import symbol_shorthand
from blockbot import BlockBot, LANDMARK_TAG
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
import matplotlib.pyplot as plt
from matplotlib import patches


def plot_point2_on_axes(axes, point, P=None, color=None):
    if color is not None:
        axes.plot([point[0]], [point[1]], marker='.', color=color, markersize=8)
    else:
        axes.plot([point[0]], [point[1]], marker='.', markersize=8)
    if P is not None:
        w, v = np.linalg.eigh(P)
        # this corresponds to 95%
        k = 2.447746830681

        angle = np.arctan2(v[1, 0], v[0, 0])
        e1 = patches.Ellipse(point,
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
            ax, result.atPoint2(key), marginals.marginalCovariance(key))

    plt.axis('equal')
    plt.savefig('localization_prob.png')


def print_result_point2(result, marginals):
    for key in result.keys():
        print(f"Point[{key}]:\n"
              f"  Location: {result.atPoint2(key)}, \n"
              f"  Covariance: \n{marginals.marginalCovariance(key)}\n" )


def start_localiztion_demo():
    bot = BlockBot()
    bot.base.reset_odom()

    L = symbol_shorthand.L
    X = symbol_shorthand.X

    # Create noise models
    PRIOR_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01], dtype=float))
    ODOMETRY_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01], dtype=float))
    LANDMARK_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.5, 0.5], dtype=float))

    graph = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()

    graph.add(gtsam.PriorFactorPoint2(X(0), gtsam.Point2(0, 0), PRIOR_NOISE))
    initial_estimate.insert(X(0), gtsam.Point2(0, 0))

    result = None
    marginals = None
    # Move in segments
    odom_history = [(0, 0)]
    for i in range(5):
        time_idx = int(i + 1)
        bot.base.move(.1, 0, 4)

        # Process odometry
        x, y, z = bot.base.get_odom()
        
        prev_x, prev_y = odom_history[-1]
        odom_history.append((x, y))

        dx = x - prev_x
        dy = y - prev_y
        graph.add(gtsam.BetweenFactorPoint2(X(time_idx - 1), X(time_idx), gtsam.Point2(dx, dy), ODOMETRY_NOISE))
        initial_estimate.insert(X(i+1), gtsam.Point2(x, y))

        # Process any visible landmarks
        landmarks = [tag for tag in bot.tags_data if tag.id[0] == LANDMARK_TAG]
        for l in landmarks:
            l_pos = l.pose.pose.pose.position
            # graph.add(gtsam.BetweenFactorPoint2(X(time_idx)), L(l.id[0]), gtsam.Point2(l_pos.x, l_dy), LANDMARK_NOISE)


        # Parameters for the optimization
        parameters = gtsam.GaussNewtonParams()
        parameters.setRelativeErrorTol(1e-5)
        parameters.setMaxIterations(1000)
        optimizer = gtsam.GaussNewtonOptimizer(graph, initial_estimate, parameters)
        result = optimizer.optimize()
        marginals = gtsam.Marginals(graph, result)

        print("Estimated position: ")
        print(result.atPoint2(X(i+1)))
        print("Covariance:")
        print(marginals.marginalCovariance(X(i+1)))
        print("Odometry measurement")
        print(x, y)
        rospy.sleep(2)

    # Printing and Plotting
    plt.figure(figsize=(8, 8))
    # plt.plot(x_true, y_true, ':m', alpha=0.3)
    # plt.plot(landmarks[:, 0], landmarks[:, 1], 'mo', alpha=0.2)
    # print_result_point2(result)
    plot_result_point2(result, marginals)

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