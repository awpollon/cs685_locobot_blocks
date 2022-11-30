import math
import numpy as np

from pid_controller import LocobotPIDController


def calc_angle_dist(theta_1, theta_2):
    theta_rel = theta_1 - theta_2

    theta_rel = theta_rel % (2 * np.pi)
    if theta_rel > math.pi:
        theta_rel -= 2 * np.pi

    if theta_rel < -math.pi:
        theta_rel += 2 * np.pi

    return theta_rel


class LocobotController():
    '''Controller for locobot'''
    TRAVEL_ACCEPTANCE_RADIUS = 0.02
    STOP_ACCEPTANCE_RADIUS = 0.02
    GOAL_THETA_MARGIN = math.pi/32
    HEADING_THRESHOLD = math.pi/8

    MIN_THETA_VEL = math.pi/18

    def __init__(self, goal_pose=((0, 0, 0)), verbose=True) -> None:
        self.v = verbose
        self.reset_goal(goal_pose)

    def reset_goal(self, goal_pose):
        '''If angle is None, just reach coordinate and then goal is reached'''
        self.goal_pose = goal_pose
        self.goal_reached = False

        self.goal_point_reached = False

        self.x_vel_controller = LocobotPIDController(KP=0.5, KD=0, verbose=self.v)
        self.theta_vel_controller = LocobotPIDController(KP=0.7, KI=.1, KD=.1, verbose=self.v)

        self.x_vel_pose_controller = LocobotPIDController(KP=0.4, KD=0.1, verbose=self.v)
        self.theta_vel_pose_controller = LocobotPIDController(KP=0.7, KI=.1, KD=.1, verbose=self.v)

    def step(self, current_pose):
        if self.goal_reached:
            return 0, 0

        pos_x, pos_y, pos_theta = current_pose

        if self.v:
            print(f"Current pose: {current_pose}")
            print(f"Goal pose: {self.goal_pose}")

        (g_x, g_y, g_theta) = self.goal_pose

        # Check current distance
        dist = self.euclidean_distanced_to_goal(current_pose)
        if self.v:
            print(f"Dist: {dist}")

        # Default to stop
        x_vel = 0
        theta_vel = 0

        if abs(dist) <= self.TRAVEL_ACCEPTANCE_RADIUS or self.goal_point_reached:
            self.goal_point_reached = True

            if g_theta is None:
                # No angle pose set, goal is reached
                self.goal_reached = True
                return 0, 0

            # Within x, y margin, now rotate to match goal pose
            pose_theta_diff = calc_angle_dist(g_theta, pos_theta)
            if self.v:
                print(f'Pose theta diff {pose_theta_diff}')
            if abs(pose_theta_diff) <= self.GOAL_THETA_MARGIN:
                # Within pose theta margin, goal reached
                self.goal_reached = True
                x_vel = 0
                theta_vel = 0
            else:
                if self.v:
                    print(f"Rotating to goal pose only, theta_diff: {pose_theta_diff}")
                # x_vel = self.x_vel_pose_controller.step(dist)
                x_vel = 0
                theta_vel = self.theta_vel_pose_controller.step(pose_theta_diff)
        else:
            # Still not at goal position
            theta_rel = calc_angle_dist(np.arctan2(g_y - pos_y, g_x - pos_x), pos_theta)

            # Move towards goal position, adjusting for small heading errors
            if self.v:
                print(f"Moving to goal, dist: {dist}, theta_diff: {theta_rel}")
            # Only move forward when traveling
            x_vel = self.x_vel_controller.step(abs(dist))
            theta_vel = self.theta_vel_controller.step(theta_rel)

        # if 0 < abs(theta_vel) < self.MIN_THETA_VEL:
        #     theta_vel = self.MIN_THETA_VEL * abs(theta_vel) / theta_vel

        return x_vel, theta_vel

    def euclidean_distanced_to_goal(self, current_pose):
        (g_x, g_y, _) = self.goal_pose
        x, y, _ = current_pose
        dist = np.sqrt((g_y - y) ** 2 + (g_x - x) ** 2)

        if g_x < x:
            dist *= -1

        return dist

