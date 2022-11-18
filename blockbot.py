import math
import rospy
import numpy as np
from enum import Enum
from apriltag_ros.msg import AprilTagDetectionArray
from interbotix_xs_modules.locobot import InterbotixLocobotXS
from localizer import BlockBotLocalizer


class RobotActionState(Enum):
    WAIT = 0
    SEARCH_FOR_BLOCK = 1
    PICK_UP_BLOCK = 2
    MOVE_TO_GOAL = 3
    MOVE_FORWARD = 4
    RELEASE_BLOCK = 5


BLOCK_TAGS = [91, 685]
LANDMARK_TAGS = [680, 681, 682, 683, 684]
BIN_TAG = 686
TAGS = [*BLOCK_TAGS, *LANDMARK_TAGS, BIN_TAG]

ROTATION_INCREMENT = math.pi/20.0
MOVE_INCREMENT = 0.03

CAMERA_SETTINGS = {"tilt": 1, "pan": 0, "height": 0.45}

GRABBABLE_APRILTAG_Z = 0.5
GRABBABLE_MARGIN = [-0.01, 0.01]


def calc_velocities(dist, theta_rel, K_vel=0.3, K_theta=0.5):
    MAX_X_VEL = .2
    MIN_X_VEL = .03

    MAX_THETA_VEL = math.pi/4
    MIN_THETA_VEL = math.pi/16

    x_vel = min(K_vel * dist * ((math.pi - abs(theta_rel)) / math.pi), MAX_X_VEL)
    theta_vel = min(K_theta * theta_rel, MAX_THETA_VEL)

    # print(x_vel, theta_vel)
    return x_vel, theta_vel


class BlockBot(InterbotixLocobotXS):
    def __init__(self, align_camera=False) -> None:
        super().__init__("locobot_px100", "mobile_px100")
        rospy.Subscriber("/tag_detections",
                         AprilTagDetectionArray, self.get_tag_data)

        self.tags_data = []
        self.block_position = None
        self.bin_position = None
        self.found_block = False
        self.action_state = RobotActionState.WAIT
        self.initialize_robot(align_camera)

    def initialize_robot(self, align_camera):
        if align_camera:
            self.camera.move("pan", CAMERA_SETTINGS["pan"])
            self.camera.move("tilt", CAMERA_SETTINGS["tilt"])
        self.arm.go_to_sleep_pose()
        self.reset_base_pose()

        # fix any erroneous gripper positions
        self.gripper.close()
        self.gripper.open()

    def reset_base_pose(self):
        '''Resets pose estimate, odometry, and GTSAM data. 
        Call this instead of reset_odom()'''
        self.estimated_pose = (0, 0, 0)
        self.base.reset_odom()
        self.localizer = BlockBotLocalizer(self.base.get_odom())
        self.estimated_pose = self.localizer.estimated_pose

    def update_position_estimate(self):
        '''Update localizer with current odometry and observed landmarks'''
        odom = self.base.get_odom()
        landmarks = [
            tag for tag in self.tags_data if tag.id[0] in LANDMARK_TAGS]

        self.localizer.add_observation(odom, landmarks)
        self.localizer.optmize()
        self.estimated_pose = self.localizer.estimated_pose

        print("Odometry measurement")
        print(odom)

        print("Covariance:")
        print(self.localizer.current_covariance)
        print("Estimated pose: ")
        print(self.localizer.estimated_pose)

    def get_tag_data(self, data):
        self.tags_data = [tag for tag in data.detections if tag.id[0] in TAGS]

        block_tag = [tag for tag in data.detections if tag.id[0] in BLOCK_TAGS]
        if len(block_tag) > 0:
            self.block_position = block_tag[0].pose.pose.pose.position
        else:
            self.block_position = None

        bin_tag = [tag for tag in data.detections if tag.id[0] == BIN_TAG]
        if len(bin_tag) > 0:
            self.bin_position = bin_tag[0].pose.pose.pose.position
        else:
            self.bin_position = None

    def grab_block(self):
        self.action_state = RobotActionState.PICK_UP_BLOCK
        self.base.move(0, 2*ROTATION_INCREMENT, 1)
        self.arm.go_to_home_pose()
        self.arm.set_ee_cartesian_trajectory(z=-0.25)
        self.gripper.close()
        self.arm.go_to_sleep_pose()
        self.action_state = RobotActionState.WAIT

    def release_block(self):
        self.action_state = RobotActionState.RELEASE_BLOCK
        self.arm.go_to_home_pose()
        self.base.move(0.15, 0, 1)
        self.gripper.open()
        self.base.move(-0.15, 0, 1)
        self.arm.go_to_sleep_pose()
        self.action_state = RobotActionState.WAIT

    def find_goal(self, type="block"):
        self.action_state = RobotActionState.SEARCH_FOR_BLOCK
        for _ in range(500):
            if type == "block":
                pos = self.block_position
            elif type == "bin":
                pos = self.bin_position

            if not pos:  # No block in view
                self.base.move(0, 2 * ROTATION_INCREMENT, 0.5)
            else:  # Block in view
                if GRABBABLE_MARGIN[0] < pos.x < GRABBABLE_MARGIN[1]: # Block in grabbable margin
                    if abs(pos.z - GRABBABLE_APRILTAG_Z) > 0.01:  # Block is far away
                        self.base.move(MOVE_INCREMENT, 0, 0.5)
                    else:  # Block is grabbable
                        return True
                else:  # Block not in grabbable margin
                    self.base.move(
                        0, ROTATION_INCREMENT if pos.x < 0 else -ROTATION_INCREMENT, 0.5)
        self.action_state = RobotActionState.WAIT

    def move_to_goal(self, goal_pose=(0, 0, 0)):
        self.action_state = RobotActionState.MOVE_TO_GOAL
        MAX_MOVES = 500
        GOAL_DIST_MARGIN = 0.01
        GOAL_THETA_MARGIN = math.pi/16
        goal_reached = False

        r = rospy.Rate(10)
        for i in range(MAX_MOVES):
            self.update_position_estimate()

            pos_x = self.estimated_pose.x()
            pos_y = self.estimated_pose.y()
            pos_theta = self.estimated_pose.theta()

            (g_x, g_y, g_theta) = goal_pose

            # Check current distance
            dist = np.sqrt((g_y - pos_y) ** 2 + (g_x - pos_x) ** 2)
            if abs(dist) <= GOAL_DIST_MARGIN:
                # Stop moving, rotate to goal pose
                #TODO: Rotate
                goal_reached = True
                break

            theta_rel = np.arctan2(g_y - pos_y, g_x - pos_x) - pos_theta
            #TODO: Check this
            theta_rel = theta_rel % (2 * np.pi)
            if theta_rel > math.pi:
                theta_rel -= 2 * np.pi

            x_vel, theta_vel = calc_velocities(dist, theta_rel)
            print(f'Velocities: {x_vel} {theta_vel}')
            self.base.command_velocity(x_vel, theta_vel)
            r.sleep()
        self.action_state = RobotActionState.WAIT
        if i+1 == MAX_MOVES:
            print(f"Moves limit reached:{MAX_MOVES}")

    def execute_sequence(self):
        if self.find_goal("block"):
            self.grab_block()
            if self.find_goal("bin"):
                self.release_block()


if __name__ == "__main__":
    blockbot = BlockBot(True)
    blockbot.execute_sequence()
