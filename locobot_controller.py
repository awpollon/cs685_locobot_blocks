import math
import numpy as np

from pid_controller import LocobotPIDController


def calc_angle_dist(theta_1, theta_2):
    theta_rel = theta_1 - theta_2

    theta_rel = theta_rel % (2 * np.pi)
    if theta_rel > math.pi:
        theta_rel -= 2 * np.pi
    
    return theta_rel


class LocobotController():
    '''Controller for locobot'''
    GOAL_DIST_MARGIN = 0.01
    GOAL_THETA_MARGIN = math.pi/96
    HEADING_THRESHOLD = math.pi/8

    def __init__(self, goal_pose=((0, 0, 0))) -> None:
        self.reset_goal(goal_pose)

    def reset_goal(self, goal_pose):
        self.goal_point_reached = False
        self.goal_pose = goal_pose
        self.goal_reached = False

        self.heading_controller = LocobotPIDController()
        self.movement_controller = LocobotPIDController()
        self.pose_angle_controller = LocobotPIDController()

    def step(self, current_pose):
        if self.goal_reached:
            return 0, 0

        pos_x = current_pose.x()
        pos_y = current_pose.y()
        pos_theta = current_pose.theta()

        print(f"Current pose: {current_pose}")
        print(f"Goal pose: {self.goal_pose}")

        (g_x, g_y, g_theta) = self.goal_pose
        
        # Check current distance
        dist = self.euclidean_distanced_to_goal(current_pose)
        print(f"Dist: {dist}")
        
        if abs(dist) <= self.GOAL_DIST_MARGIN or self.goal_point_reached:
            self.goal_point_reached = True
            # Within x, y margin, now rotate to match goal pose
            pose_theta_diff = calc_angle_dist(g_theta, pos_theta)
            print(f'Pose theta diff {pose_theta_diff}')
            if abs(pose_theta_diff) <= self.GOAL_THETA_MARGIN:
                # Within pose theta margin, goal reached
                self.goal_reached = True
                return 0, 0

            else:
                print(f"Rotating to goal pose only, theta_diff: {pose_theta_diff}")
                return self.pose_angle_controller.step(0, pose_theta_diff)
        else:
            # Still not at goal position
            theta_rel = calc_angle_dist(np.arctan2(g_y - pos_y, g_x - pos_x), pos_theta)
            if abs(theta_rel) > self.HEADING_THRESHOLD:
                # Only rotate towards goal
                print(f"Rotating to goal heading, theta_diff: {theta_rel}")
                return self.heading_controller.step(0, theta_rel)
            else:
                # Move towards goal position, adjusting for small heading errors
                print(f"Moving to goal, dist: {dist}, theta_diff: {theta_rel}")
                return self.movement_controller.step(dist, theta_rel)

    def euclidean_distanced_to_goal(self, current_pose):
        (g_x, g_y, _) = self.goal_pose
        return np.sqrt((g_y - current_pose.y()) ** 2 + (g_x - current_pose.x()) ** 2)
