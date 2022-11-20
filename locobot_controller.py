import math
from blockbot import BlockBot
import numpy as np


def calc_velocities(dist, theta_rel, K_vel=0.3, K_theta=0.5):
    MAX_X_VEL = .2
    MIN_X_VEL = .03

    MAX_THETA_VEL = math.pi/4
    MIN_THETA_VEL = math.pi/16

    x_vel = min(K_vel * dist * ((math.pi - abs(theta_rel)) / math.pi), MAX_X_VEL)
    theta_vel = min(K_theta * theta_rel, MAX_THETA_VEL)

    if theta_rel > math.pi/8:
        x_vel = 0

    # print(x_vel, theta_vel)
    return x_vel, theta_vel


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
                self.__rotate_only(pose_theta_diff)
        else:
            # Still not at goal position
            theta_rel = calc_angle_dist(np.arctan2(g_y - pos_y, g_x - pos_x), pos_theta)
            if theta_rel > self.HEADING_THRESHOLD:
                # Only rotate towards goal
                self.__rotate_only(theta_rel)
            else:
                # Move towards goal position, adjusting for small heading errors
                self.__move_towards_goal(dist, theta_rel)

    def __rotate_only(self, theta_rel):
        _, theta_vel = calc_velocities(0, theta_rel)
        self.command(0, theta_vel)
    
    def __move(self, dist, theta_rel):
        x_vel, theta_vel = calc_velocities(dist, theta_rel)
        self.command(x_vel, theta_vel)

    def __stop(self):
        self.command(0, 0)

    def __command(self, x_vel, theta_vel):
        print(f'Velocities: {x_vel} {theta_vel}')
        self.bot.base.command_velocity(x_vel, theta_vel)

    def euclidean_distanced_to_goal(self):
        (g_x, g_y, _) = self.goal_pose
        current_pose = self.bot.estimated_pose

        return np.sqrt((g_y - current_pose.y()) ** 2 + (g_x - current_pose.x()) ** 2)
