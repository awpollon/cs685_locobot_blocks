#!/bin/python3

import os
import math
from interbotix_xs_modules.locobot import InterbotixLocobotXS


class BlockBot:
    def __init__(self) -> None:
        self.locobot = InterbotixLocobotXS("locobot_" + os.environ["LOCOBOT_MODEL"], "mobile_" + os.environ["LOCOBOT_MODEL"])

    def pick_up_block(self):
        if os.environ["LOCOBOT_MODEL"] == 'wx250s':
            z_move = -0.39
        else:
            z_move = -0.25

        self.locobot.arm.go_to_home_pose()
        self.locobot.gripper.open()
        self.locobot.arm.set_ee_cartesian_trajectory(z=z_move)
        self.locobot.gripper.close()
        self.locobot.arm.go_to_home_pose()


def main():
    bot = BlockBot()
    bot.locobot.base.move(0, math.pi/8, 8)
    bot.pick_up_block()

    bot.locobot.base.move(0, -math.pi/8, 8)
    bot.locobot.gripper.open()

    bot.locobot.arm.go_to_sleep_pose()


if __name__ == '__main__':
    main()
