import math
from blockbot import BlockBot
import numpy as np

from pid_controller import LocobotPIDController


def calc_angle_dist(theta_1, theta_2):
    theta_rel = theta_1 - theta_2
    #TODO: Check this
    theta_rel = theta_rel % (2 * np.pi)
    if theta_rel > math.pi:
        theta_rel -= 2 * np.pi
    
    return theta_rel


class LocobotController():
    '''PID Controller for locobot'''
    GOAL_DIST_MARGIN = 0.01
    GOAL_THETA_MARGIN = math.pi/32
    HEADING_THRESHOLD = math.pi/8

    def __init__(self, locobot: BlockBot, goal_pose) -> None:
        self.bot = locobot
        self.goal_reached = False
        self.goal_pose = goal_pose

        self.heading_controller = LocobotPIDController()
        self.movement_controller = LocobotPIDController()
        self.pose_angle_controller = LocobotPIDController()

    def set_goal(self, goal_pose):
        self.goal_pose = goal_pose
        self.goal_reached = False

    def step(self):
        if self.goal_reached:
            return

        self.bot.update_position_estimate()
        pos_x = self.bot.estimated_pose.x()
        pos_y = self.bot.estimated_pose.y()
        pos_theta = self.bot.estimated_pose.theta()

        (g_x, g_y, g_theta) = self.goal_pose
        
        # Check current distance
        dist = self.euclidean_distanced_to_goal()
        
        if abs(dist) <= self.GOAL_DIST_MARGIN:
            # Within x, y margin, now rotate to match goal pose
            pose_theta_diff = calc_angle_dist(g_theta, pos_theta)
            print(f'Pose theta diff {pose_theta_diff}')
            if abs(pose_theta_diff) <= self.GOAL_THETA_MARGIN:
                # Within pose theta margin, goal reached
                print("Goal reached")
                self.__stop()
                self.goal_reached = True
                return

            else:
                self.pose_angle_controller.step(0, pose_theta_diff)
        else:
            # Still not at goal position
            theta_rel = calc_angle_dist(np.arctan2(g_y - pos_y, g_x - pos_x), pos_theta)
            if theta_rel > self.HEADING_THRESHOLD:
                # Only rotate towards goal
                self.heading_controller.step(0, theta_rel)
            else:
                # Move towards goal position, adjusting for small heading errors
                self.movement_controller.step(dist, theta_rel)

    def __stop(self):
        self.bot.base.command_velocity(0, 0)

    def euclidean_distanced_to_goal(self):
        (g_x, g_y, _) = self.goal_pose
        current_pose = self.bot.estimated_pose

        return np.sqrt((g_y - current_pose.y()) ** 2 + (g_x - current_pose.x()) ** 2)
