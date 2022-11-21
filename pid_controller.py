import math
import rospy


class LocobotPIDController:
    MAX_X_VEL = .3
    MAX_THETA_VEL = math.pi/4

    def __init__(self) -> None:
        self.last_step_time = None
        self.last_dist = None
        self.last_theta = None

        self.KP_vel = 0.7
        self.KP_theta = 1

        self.KI_vel = 0
        self.KI_theta = .01

        self.KD_vel = 0.05
        self.KD_theta = 0.1
        
        self.x_it = 0
        self.theta_it = 0
        
    def step(self, dist, theta_rel):
        # Determine change in time
        time = rospy.get_time()
        dt = time - self.last_step_time if self.last_step_time else 0
        
        x_p = self.KP_vel * dist
        theta_p = self.KP_theta * theta_rel

        # Dampen x based on magnitude of theta
        x_p = x_p * ((math.pi - abs(theta_rel)) / math.pi)

        print(f"dt={dt}")
        self.x_it += (dist * self.KI_vel * dt)
        self.theta_it += (theta_rel * self.KI_theta * dt)

        x_d = self.KD_vel * ((dist - self.last_dist) / dt) if self.last_dist else 0
        theta_d = self.KD_theta * ((theta_rel - self.last_theta) / dt) if self.last_theta else 0

        self.last_step_time = time
        self.last_dist = dist
        self.last_theta = theta_rel

        raw_x_vel = x_p + self.x_it + x_d
        print(f"raw_x_vel = {x_p} + {self.x_it} + {x_d} = {raw_x_vel}")
        
        raw_theta_vel = theta_p + self.theta_it + theta_d
        print(f"raw_theta_vel = {theta_p} + {self.theta_it} + {theta_d} = {raw_theta_vel}")

        x_vel = max(min(raw_x_vel, self.MAX_X_VEL), -self.MAX_X_VEL)
        theta_vel = max(min(raw_theta_vel, self.MAX_THETA_VEL), -self.MAX_THETA_VEL)

        return x_vel, theta_vel

