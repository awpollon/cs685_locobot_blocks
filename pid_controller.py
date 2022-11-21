import math
import rospy


class LocobotPIDController:
    MAX_X_VEL = .2
    MAX_THETA_VEL = math.pi/4

    def __init__(self) -> None:
        # self.bot = bot
        self.last_step_time = 0
        self.last_dist = None
        self.last_theta = None

        self.KP_vel = 0.3
        self.KP_theta = 0.2

        self.KI_vel = 0.1
        self.KI_theta = 0.1

        self.KD_vel = 0
        self.KD_theta = 0
        
        self.x_it = 0
        self.theta_it = 0
        
    def step(self, dist, theta_rel):
        # Determine change in time
        time = rospy.get_time()
        dt = time - self.last_step_time
        
        x_p = self.KP_vel * dist
        theta_p = self.KP_theta * theta_rel

        # Dampen x based on magnitude of theta
        x_p = x_p * ((math.pi - abs(theta_rel)) / math.pi)

        self.x_it += (dist * self.KI_vel * dt)
        self.theta_it += (theta_rel * self.KI_theta * dt)

        x_d = self.KD_vel * ((self.last_dist - dist) / dt) if self.last_dist else 0
        theta_d = self.KD_theta * ((self.last_theta - theta_rel) / dt) if self.last_theta else 0

        self.last_step_time = time
        self.last_dist = dist
        self.last_theta = theta_rel

        raw_x_vel = x_p + self.x_it + x_d
        raw_theta_vel = theta_p + self.theta_it + theta_d

        x_vel = min(raw_x_vel, self.MAX_X_VEL)
        theta_vel = min(raw_theta_vel, self.MAX_THETA_VEL)

        return x_vel, theta_vel

