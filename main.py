#!/bin/python3

import math
import os

from interbotix_xs_modules.locobot import InterbotixLocobotXS


class BlockBot(InterbotixLocobotXS):

    def pick_up_block(self):
        if os.environ["LOCOBOT_MODEL"] == 'wx250s':
            z_move = -0.39
        else:
            z_move = -0.25

        self.arm.go_to_home_pose()
        self.gripper.open()
        self.arm.set_ee_cartesian_trajectory(z=z_move)
        self.gripper.close()
        self.arm.go_to_home_pose()


def main():
    locobot = BlockBot("locobot_" + os.environ["LOCOBOT_MODEL"], "mobile_" + os.environ["LOCOBOT_MODEL"])

    locobot.base.move(0, math.pi/12, 12)
    locobot.pick_up_block()

    locobot.base.move(0, -math.pi/12, 12)
    locobot.gripper.open()

    locobot.arm.go_to_sleep_pose()


if __name__ == '__main__':
    main()
