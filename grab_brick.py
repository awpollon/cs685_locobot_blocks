#!/bin/python3

import os
from interbotix_xs_modules.locobot import InterbotixLocobotXS

def GrabBrick(x_move = 0.02):
    locobot = InterbotixLocobotXS("locobot_" + os.environ["LOCOBOT_MODEL"], "mobile_" + os.environ["LOCOBOT_MODEL"])
    if os.environ["LOCOBOT_MODEL"] == 'wx250s':
        z_move = -0.39
    else:
        z_move = -0.25
    locobot.arm.go_to_home_pose()
    locobot.gripper.open()
    locobot.arm.set_ee_cartesian_trajectory(x = x_move)
    locobot.gripper.close()
    locobot.arm.go_to_sleep_pose()

if __name__=='__main__':
    main()
