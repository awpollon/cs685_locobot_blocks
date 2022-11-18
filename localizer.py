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
    np.array([0.01, 0.01, 0.01], dtype=float))
LANDMARK_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([0.1, 0.2], dtype=float))


class BlockBotLocalizer:
    def __init__(self, start=(0, 0, 0)) -> None:
        # Track pose id index
        self.current_idx = 0
        self.use_landmarks = False

        self.seen_landmarks = set()
        self.odom_history = [start]
        self.initial_estimate = gtsam.Values()

        # Init the factor graph
        self.graph = gtsam.NonlinearFactorGraph()

        start_pose = gtsam.Pose2(*start)

        self.graph.add(gtsam.PriorFactorPose2(X(0), start_pose, PRIOR_NOISE))
        self.initial_estimate.insert(X(self.current_idx), start_pose)
        self.optmize()

    def add_observation(self, odom, landmarks=[], time_idx=None):
        '''Adds a new observation. Increments time index unless one is provided'''
        if time_idx is None:
            self.current_idx += 1
            time_idx = self.current_idx

        # Add odometry measurement
        prev_odom = self.odom_history[-1]
        self.odom_history.append(odom)

        d_odom = np.subtract(odom, prev_odom)
        odom_rel_pose = gtsam.Pose2(d_odom[0], d_odom[1], d_odom[2])
        self.graph.add(gtsam.BetweenFactorPose2(X(time_idx - 1), X(time_idx),
                       odom_rel_pose, ODOMETRY_NOISE))

        self.initial_estimate.insert(
            X(time_idx), gtsam.Pose2(odom[0], odom[1], odom[2]))

        # Add landmark observations
        if self.use_landmarks:
            for l in landmarks:
                l_id = l.id[0]
                tag = l.pose.pose.pose.position
                # Assume camera is parallel to ground (tilt = 0)
                # TODO: adjust for camera tilt
                # Project the tag position to the camera center
                l_range = math.sqrt(tag.x**2 + tag.y**2 + tag.z**2)
                l_bearing = math.atan2(math.sqrt(tag.x**2 + tag.y**2), tag.z)
                self.graph.add(gtsam.BearingRangeFactor2D(X(time_idx), L(l_id),
                            gtsam.Rot2(l_bearing), l_range, LANDMARK_NOISE))

                # Add estimate for landmark if not seen before
                if l_id not in self.seen_landmarks:
                    # TODO: Better estimate based on odom, bearing, and range
                    # rng = default_rng()
                    # p = rng.normal(loc=0, scale=100, size=(2,))
                    # initial_estimate.insert(L(l_id), p)
                    self.initial_estimate.insert(L(l_id), gtsam.Pose2(0, 0, 0))

                    self.seen_landmarks.add(l_id)

    def optmize(self):
        # Parameters for the optimization
        parameters = gtsam.GaussNewtonParams()
        parameters.setRelativeErrorTol(1e-5)
        parameters.setMaxIterations(1000)
        optimizer = gtsam.GaussNewtonOptimizer(
                    self.graph, self.initial_estimate, parameters)
        result = optimizer.optimize()
        marginals = gtsam.Marginals(self.graph, result)

        # Update estimated pose
        self.estimated_pose = result.atPose2(X(self.current_idx))
        self.current_covariance = marginals.marginalCovariance(X(self.current_idx))