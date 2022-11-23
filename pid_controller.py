import rospy


class LocobotPIDController:
    def __init__(self, KP=0, KI=0, KD=0) -> None:
        self.last_step_time = None
        self.last_error = None

        self.KP = KP
        self.KI = KI
        self.KD = KD

        self.it = 0
        
    def step(self, error):
        # Determine change in time
        time = rospy.get_time()
        dt = time - self.last_step_time if self.last_step_time else 0
        
        p = self.KP * error

        print(f"dt={dt}")
        self.it += (error * self.KI * dt)

        d = self.KD * ((error - self.last_error) / dt) if self.last_error else 0

        self.last_step_time = time
        self.last_error = error

        raw_control = p + self.it + d
        print(f"raw_control = {p} + {self.it} + {d} = {raw_control}")

        return raw_control
    