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
    np.array([0.1, 0.1, math.pi/8], dtype=float))
LANDMARK_NOISE = gtsam.noiseModel.Diagonal.Sigmas(
    np.array([0.5, 0.5], dtype=float))


def calc_pos_from_bearing_range(pose, l_bearing, l_range):
    # Calc relative x and y from robot pose
    dx = l_range * np.sin(l_bearing)
    dy = l_range * np.cos(l_bearing)

    # roate opposite robot pose to get absolute dx and dy from pose
    r_x, r_y, r_theta = pose
    l_x = r_x + (dx * np.cos(-r_theta) + dy * np.sin(-r_theta))
    l_y = r_y + (dx * np.sin(r_theta) + dy * np.cos((-r_theta)))

    l_theta = r_theta + l_bearing

    return (l_x, l_y, l_theta)


class BlockBotLocalizer:
    def __init__(self, start=(0, 0, 0), use_landmarks=True) -> None:
        # Track pose id index
        self.current_idx = 0
        self.use_landmarks = use_landmarks
        self.debug = True

        self.seen_landmarks = set()
        self.odom_history = [start]
        self.initial_estimate = gtsam.Values()

        # Init the factor graph
        self.graph = gtsam.NonlinearFactorGraph()

        start_pose = gtsam.Pose2(*start)

        self.graph.add(gtsam.PriorFactorPose2(X(0), start_pose, PRIOR_NOISE))
        self.initial_estimate.insert(X(self.current_idx), start_pose)
        self.optmize()

    def add_observation(self, odom, landmarks=[], camera_tilt=0.0, time_idx=None):
        '''Adds a new observation. Increments time index unless one is provided'''
        if time_idx is None:
            self.current_idx += 1
            time_idx = self.current_idx

        # Add odometry measurement
        prev_odom = self.odom_history[-1]
        self.odom_history.append(odom)
        dx, dy, dtheta = np.subtract(odom, prev_odom)

        prev_theta = prev_odom[2]
        rel_x = dx * np.cos(prev_theta) + dy * np.sin(prev_theta)
        rel_y = dx * np.sin(-prev_theta) + dy * np.cos(prev_theta)

        odom_rel_pose = gtsam.Pose2(rel_x, rel_y, dtheta)
        print(f'Rel pose: {odom_rel_pose}')
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
                l_range = math.sqrt(tag.x**2 + tag.y**2 + tag.z**2) * np.cos(camera_tilt)
                l_bearing = math.atan2(math.sqrt(tag.x**2 + tag.y**2), tag.z)

                if tag.x > 0:
                    l_bearing *= -1
                    
                self.graph.add(gtsam.BearingRangeFactor2D(X(time_idx), L(l_id),
                            gtsam.Rot2(l_bearing), l_range, LANDMARK_NOISE))

                print(f'Landmark {l_id} range: {l_range} bearing: {l_bearing}')
                print(f'Raw z: {tag.z},x: {tag.x} y: {tag.y}')

                # Add estimate for landmark if not seen before
                if l_id not in self.seen_landmarks:
                    # Estimate landmark pos based on current odometry
                    l_pose = calc_pos_from_bearing_range(odom, l_bearing, l_range)
                    self.initial_estimate.insert(L(l_id), gtsam.Point2(l_pose[0], l_pose[1]))
                    print(f'Initial estimate for {l_id}: {l_pose[0]}, {l_pose[1]}')
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


if __name__ == "__main__":
    p = (4, 1.5, 7 * math.pi/18)
    l_range = 3
    l_bearing = -5 * math.pi/18
    calc_pos_from_bearing_range(p, l_bearing, l_range)