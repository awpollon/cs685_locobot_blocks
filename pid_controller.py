import rospy


class PIDController:
    def __init__(self, KP=0, KI=0, KD=0, verbose=True) -> None:
        self.v = verbose

        self.last_step_time = None
        self.last_error = None

        self.KP = KP
        self.KI = KI
        self.KD = KD

        self.error_sum = 0

    def step(self, error):
        # Determine change in time
        time = rospy.get_time()
        dt = time - self.last_step_time if self.last_step_time else 0

        p = self.KP * error

        if self.v:
            print(f"error={error}, dt={dt}")
        self.error_sum += error * dt

        i = self.KI * self.error_sum

        dedt = ((error - self.last_error) / dt) if self.last_error and dt != 0 else 0

        d = self.KD * dedt

        self.last_step_time = time
        self.last_error = error

        raw_control = p + i + d
        if self.v:
            print(f"raw_control = {p} + {i} + {d} = {raw_control}")

        return raw_control
