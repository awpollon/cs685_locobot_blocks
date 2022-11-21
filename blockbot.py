import math
import rospy
import numpy as np
from enum import Enum
from apriltag_ros.msg import AprilTagDetectionArray
from interbotix_xs_modules.locobot import InterbotixLocobotXS
from localizer import BlockBotLocalizer
from locobot_controller import LocobotController


class RobotActionState(Enum):
    WAIT = 0
    SEARCH_FOR_BLOCK = 1
    PICK_UP_BLOCK = 2
    MOVE_TO_GOAL = 3
    MOVE_FORWARD = 4
    RELEASE_BLOCK = 5


BLOCK_TAGS = [91, 685]
LANDMARK_TAGS = [680, 681, 682, 683, 684, 86]
BIN_TAG = 413
TAGS = [*BLOCK_TAGS, *LANDMARK_TAGS, BIN_TAG]

ROTATION_INCREMENT = math.pi/16.0
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

    if theta_rel > math.pi/8:
        x_vel = 0

    # print(x_vel, theta_vel)
    return x_vel, theta_vel


def calc_angle_dist(theta_1, theta_2):
    theta_rel = theta_1 - theta_2

    sign = 1 if theta_rel >= 0 else -1

    #TODO: Check this
    theta_rel = theta_rel % (2 * np.pi)

    if theta_rel > math.pi:
        theta_rel -= 2 * np.pi
        
    
    return theta_rel * sign


class BlockBot(InterbotixLocobotXS):
    def __init__(self, align_camera=True) -> None:
        super().__init__("locobot_px100", "mobile_px100")
        rospy.Subscriber("/tag_detections",
                         AprilTagDetectionArray, self.get_tag_data)

        self.tags_data = []
        self.block_position = None
        self.bin_position = None
        self.found_block = False
        self.controller = LocobotController()

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
        rospy.sleep(1)
        print(f"Reset odom:{self.base.get_odom()}")
        self.localizer = BlockBotLocalizer(self.base.get_odom(), use_landmarks=False)
        self.estimated_pose = self.localizer.estimated_pose

    def update_position_estimate(self):
        '''Update localizer with current odometry and observed landmarks'''
        odom = self.base.get_odom()
        landmarks = [
            tag for tag in self.tags_data if tag.id[0] in LANDMARK_TAGS]

        # tilt downward is postive
        camera_tilt = -self.camera.info['tilt']['command']
        self.localizer.add_observation(odom, landmarks, camera_tilt)
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

    def move(self, x=0, yaw=0, duration=1.0):
        '''Adapted from Interbotix API, but adding localization'''
        print(f'Moving x={x} yaw={yaw}, dur={duration}')
        time_start = rospy.get_time()
        r = rospy.Rate(10)
        # Publish Twist at 10 Hz for duration
        while (rospy.get_time() < (time_start + duration)):
            self.update_position_estimate()
            self.base.command_velocity(x, yaw)
            r.sleep()
        # After the duration has passed, stop
        self.base.command_velocity(0, 0)

    def grab_block(self):
        self.action_state = RobotActionState.PICK_UP_BLOCK
        self.move(0, 2*ROTATION_INCREMENT, 1)
        self.arm.go_to_home_pose()
        self.arm.set_ee_cartesian_trajectory(z=-0.25)
        self.gripper.close()
        self.arm.go_to_sleep_pose()
        self.action_state = RobotActionState.WAIT

    def release_block(self):
        self.action_state = RobotActionState.RELEASE_BLOCK
        self.arm.go_to_home_pose()
        self.move(0.15, 0, 1)
        self.gripper.open()
        self.move(-0.15, 0, 1)
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
                self.move(0, 2 * ROTATION_INCREMENT, 0.5)
            else:  # Block in view
                if GRABBABLE_MARGIN[0] < pos.x < GRABBABLE_MARGIN[1]: # Block in grabbable margin
                    if abs(pos.z - GRABBABLE_APRILTAG_Z) > 0.01:  # Block is far away
                        self.move(MOVE_INCREMENT, 0, 0.5)
                    else:  # Block is grabbable
                        return True
                else:  # Block not in grabbable margin
                    self.move(
                        0, ROTATION_INCREMENT if pos.x < 0 else -ROTATION_INCREMENT, 0.5)
        self.action_state = RobotActionState.WAIT

    def move_to_goal(self, goal_pose=(0, 0, 0)):
        self.action_state = RobotActionState.MOVE_TO_GOAL
        MAX_MOVES = 500
        self.controller.reset_goal(goal_pose)

        r = rospy.Rate(10)        
        for i in range(MAX_MOVES):
            # Ideally handled by controller, but avoiding circ dependecy
            self.update_position_estimate()
            if (self.controller.goal_reached):
                print("Goal reached")
                self.__command(0, 0)
                return True

            x_vel, theta_vel = self.controller.step(self.estimated_pose)
            self.__command(x_vel, theta_vel)
            r.sleep()
        self.action_state = RobotActionState.WAIT
        if i+1 == MAX_MOVES:
            print(f"Moves limit reached:{MAX_MOVES}")

        return False

    def __command(self, x_vel, theta_vel):
        print(f'Velocities: {x_vel} {theta_vel}')
        self.base.command_velocity(x_vel, theta_vel)

    def useLandmarks(self, use):
        self.localizer.use_landmarks = use
    
    def execute_sequence(self):
        if self.find_goal("block"):
            self.grab_block()
            if self.move_to_goal((0, 0, -math.pi)):
                self.release_block()
                self.move_to_goal()


if __name__ == "__main__":
    blockbot = BlockBot(True)
    blockbot.execute_sequence()
