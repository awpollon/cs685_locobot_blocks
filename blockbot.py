import math
import rospy
import numpy as np
from enum import Enum
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose2D
from interbotix_xs_modules.locobot import InterbotixLocobotXS
#TODO: Remove these from the localizer module?
from landmark_localizer.localizer import calc_pos_from_bearing_range, calc_bearing_range_from_tag
from locobot_controller import LocobotController
from pid_controller import PIDController
from kobuki_msgs.msg import BumperEvent


class RobotActionState(Enum):
    WAIT = 0
    SEARCH_FOR_BLOCK = 1
    TRAVEL_TO_BLOCK = 2
    ALIGN_WITH_BLOCK = 3
    PICK_UP_BLOCK = 4
    MOVE_TO_GOAL = 5
    RELEASE_BLOCK = 6
    RETURN_HOME = 7


MAX_X_VEL = .3
MAX_THETA_VEL = math.pi

BLOCK_TAGS = [91, 685, 686]
LANDMARK_TAGS = [680, 681, 682, 683, 684, 86]
BIN_TAG = 413
TAGS = [*BLOCK_TAGS, *LANDMARK_TAGS, BIN_TAG]

ROTATION_INCREMENT = 3*math.pi/16
SEARCH_INCREMENT_X = 0.60
BLOCK_DETECTION_RANGE = 1.00
X_BOUNDARY = 3.00

CAMERA_SETTINGS = {"tilt": 1, "search_tilt": 3*math.pi/16, "pan": 0, "height": 0.45}

BLOCK_TRAVEL_RADIUS = 0.40

GRABBING_RADIUS = 0.36
GRABBING_BEARING = 0
GRABBING_ERROR_DISTANCE = 0.05

ALIGN_BEARING_ACCEPTANCE = 0.02
ALIGN_RADIUS_ACCEPTANCE = 0.01

CONTROL_LOOP_LIMIT = 500


def calc_angle_dist(theta_1, theta_2):
    theta_rel = theta_1 - theta_2
    theta_rel = theta_rel % (2 * np.pi)

    if theta_rel > math.pi:
        theta_rel -= 2 * np.pi

    return theta_rel


class BlockBot(InterbotixLocobotXS):
    def __init__(self, align_camera=True, verbose=True) -> None:
        super().__init__("locobot_px100", "mobile_px100")
        rospy.Subscriber(
            "/tag_detections",
            AprilTagDetectionArray,
            self.get_tag_data
        )

        rospy.Subscriber(
            "landmark_slam/estimated_pose",
            Pose2D,
            self.update_position_estimate
        )

        rospy.Subscriber(
            "/locobot/mobile_base/events/bumper",
            BumperEvent,
            self.handle_bumper_event
        )

        self.v = verbose
        self.tags_data = []
        self.block_tag_data = None
        self.bin_tag_data = None
        self.found_block = False
        self.use_landmarks = True
        self.controller = LocobotController(verbose=self.v)

        self.halt = False

        self.action_state = RobotActionState.WAIT
        self.initialize_robot(align_camera)
        self.estimated_pose = (0, 0, 0)

    def initialize_robot(self, align_camera):
        if align_camera:
            self.camera.move("pan", CAMERA_SETTINGS["pan"])
            self.camera.move("tilt", CAMERA_SETTINGS["search_tilt"])
        self.arm.go_to_sleep_pose()
        self.reset_base_pose()

        # fix any erroneous gripper positions
        self.gripper.close()
        self.gripper.open()

    def reset_base_pose(self):
        '''Resets pose estimate, odometry, and GTSAM data.
        Call this instead of reset_odom()'''
        self.base.reset_odom()
        rospy.sleep(1)
        print(f"Reset odom:{self.base.get_odom()}")

    def update_position_estimate(self, pose: Pose2D):
        self.estimated_pose = pose

    def get_tag_data(self, data):
        self.tags_data = [tag for tag in data.detections if tag.id[0] in TAGS]

        block_tag = [tag for tag in data.detections if tag.id[0] in BLOCK_TAGS]
        if len(block_tag) > 0:
            self.block_tag_data = block_tag[0].pose.pose.pose.position

        bin_tag = [tag for tag in data.detections if tag.id[0] == BIN_TAG]
        if len(bin_tag) > 0:
            self.bin_tag_data = bin_tag[0].pose.pose.pose.position
        else:
            self.bin_tag_data = None

    def move(self, x=0, yaw=0, duration=1.0):
        '''Adapted from Interbotix API'''
        if self.v:
            print(f'Moving x={x} yaw={yaw}, dur={duration}')
        time_start = rospy.get_time()
        r = rospy.Rate(10)
        # Publish Twist at 10 Hz for duration
        while (rospy.get_time() < (time_start + duration)):
            self.__command(x, yaw)
            r.sleep()
        # After the duration has passed, stop
        self.__command(0, 0)

    def grab_block(self):
        self.action_state = RobotActionState.PICK_UP_BLOCK
        self.arm.go_to_home_pose()
        self.arm.set_ee_cartesian_trajectory(z=-0.25)

        # Move forward a touch to allow for some error
        self.move(GRABBING_ERROR_DISTANCE, 0, 1.5)
        self.gripper.close()
        self.arm.go_to_sleep_pose()
        self.block_tag_data = None
        self.action_state = RobotActionState.WAIT
        return True

    def release_block(self):
        self.action_state = RobotActionState.RELEASE_BLOCK
        self.arm.go_to_home_pose()
        self.move(0.15, 0, 1)
        self.gripper.open()
        self.move(-0.15, 0, 1)
        self.arm.go_to_sleep_pose()
        self.action_state = RobotActionState.WAIT

    def find_goal(self, type="block"):
        if self.v:
            print("Searching for block.")

        self.action_state = RobotActionState.SEARCH_FOR_BLOCK
        self.camera.move("tilt", CAMERA_SETTINGS["search_tilt"])

        for i in range(CONTROL_LOOP_LIMIT):
            if type == "block":
                pos = self.block_tag_data
            elif type == "bin":
                pos = self.bin_tag_data

            if pos:
                # Calculate block bearing and range, return
                camera_tilt = self.get_camera_tilt()
                block_bearing, block_range = calc_bearing_range_from_tag(pos, camera_tilt)

                # If block is too far, skip for now
                if block_range < BLOCK_DETECTION_RANGE:
                    return block_bearing, block_range
                
                print(f"Block detected, but too far. Range: {block_range}")

            # No block in current view, go to next
            x, y, _ = self.get_estimated_pose()

            if i % 3 == 0:
                self.move_to_goal((x, y, ROTATION_INCREMENT))

            elif i % 3 == 1:
                self.move_to_goal((x, y, -ROTATION_INCREMENT))
            
            else:
                if x > X_BOUNDARY:
                    print(f"X boundary reached: {X_BOUNDARY}")
                    return None
                self.move_to_goal((x, y, 0))
                self.move_to_goal((x + SEARCH_INCREMENT_X, y, 0))
            
            rospy.sleep(1)

        print("Block search limit reached")
        return None

    def travel_to_block(self, block_bearing_range):
        self.action_state = RobotActionState.TRAVEL_TO_BLOCK
        est_block_x, est_block_y = self.estimate_block_position(block_bearing_range)

        block_bearing, _ = block_bearing_range

        # Move to point near block, don't rotate to any particular goal angle
        dx = BLOCK_TRAVEL_RADIUS * np.sin(math.pi / 2 - block_bearing)
        dy = BLOCK_TRAVEL_RADIUS * np.cos(math.pi / 2 - block_bearing)

        target_pose = (est_block_x - dx, est_block_y - dy, None)
        if self.v:
            print(f"Starting travel to block. Target pose: {target_pose}")
            print(f"Block bearing/range: {block_bearing_range}")

        if not self.move_to_goal(target_pose):
            print("Unable to reach block")
            self.action_state = RobotActionState.WAIT
            return False
        else:
            return True

    def get_block_bearing_range(self):
        camera_tilt = self.get_camera_tilt()

        pos = self.block_tag_data
        if not pos:
            # print("Block out of sight")
            return (None, None)

        print(f"Block tag: {pos}")
        print(f"Camera tilt: {camera_tilt}")
        block_bearing, block_range = calc_bearing_range_from_tag(pos, camera_tilt)
        print(f"Block bearing: {block_bearing}, range: {block_range}")

        return block_bearing, block_range

    def align_with_block(self):
        if self.v:
            print("Starting block alignment.")

        self.camera.move("tilt", CAMERA_SETTINGS["tilt"])

        # Let camera finish tilting
        rospy.sleep(2)
        self.block_tag_data = None

        self.action_state = RobotActionState.ALIGN_WITH_BLOCK

        x_align_controller = PIDController(KP=0.4, KI=.05, KD=0.05, verbose=self.v)
        theta_align_controller = PIDController(KP=0.7, KI=.01, KD=.05, verbose=self.v)

        r = rospy.Rate(10)
        for _ in range(CONTROL_LOOP_LIMIT):
            block_bearing, block_range = self.get_block_bearing_range()

            if block_bearing and block_range:
                block_bearing += GRABBING_BEARING
                block_range -= GRABBING_RADIUS

                print(f"Modified block_range: {block_range}")
                print(f"Modified block_bearing: {block_bearing}")


                if abs(block_bearing) < ALIGN_BEARING_ACCEPTANCE and abs(block_range) < ALIGN_RADIUS_ACCEPTANCE:
                    print("Aligned")
                    self.__command(0, 0)
                    return True
                else:
                    if abs(block_bearing) < ALIGN_BEARING_ACCEPTANCE:
                        theta = 0
                    else:
                        theta = theta_align_controller.step(block_bearing)

                    if abs(block_range) < ALIGN_RADIUS_ACCEPTANCE:
                        x = 0
                    else:
                        x = x_align_controller.step(block_range)

                    self.__command(x, theta)
            r.sleep()

        print("Loop limit reached in align_with_block")
        return False

    def move_to_goal(self, goal_pose=(0, 0, 0)):
        print(f"Starting move to goal: {goal_pose}")
        self.action_state = RobotActionState.MOVE_TO_GOAL
        self.controller.reset_goal(goal_pose)

        r = rospy.Rate(10)
        for _ in range(CONTROL_LOOP_LIMIT):
            # Ideally handled by controller, but avoiding circ dependecy
            if (self.controller.goal_reached):
                print("Goal reached")
                self.__command(0, 0)
                return True

            x_vel, theta_vel = self.controller.step(self.get_estimated_pose())
            self.__command(x_vel, theta_vel)
            r.sleep()

        self.action_state = RobotActionState.WAIT
        print(f"Moves limit reached trying to move to goal: {goal_pose}")
        return False

    def __command(self, raw_x_vel, raw_theta_vel):
        if self.halt:
            print("HALTING")
            self.base.command_velocity(0, 0)
            self.halt = False
            raise SystemError("Halt issued")

        x_vel = max(min(raw_x_vel, MAX_X_VEL), -MAX_X_VEL)
        theta_vel = max(min(raw_theta_vel, MAX_THETA_VEL), -MAX_THETA_VEL)

        if self.v:
            print(f'Velocities: {x_vel} {theta_vel}')

        if x_vel <= -MAX_X_VEL:
            raise ValueError("X vel is too negative")

        self.base.command_velocity(x_vel, theta_vel)

    def set_use_landmarks(self, use):
        self.use_landmarks = use

    def execute_sequence(self):
        for _ in range(2):
            block_bearing_range = self.find_goal("block")
            if block_bearing_range is None:
                return

            found_pose = self.get_estimated_pose()
            print(f"Saving pose: {found_pose}")

            if not self.travel_to_block(block_bearing_range):
                return

            if not self.align_with_block():
                return

            if not self.grab_block():
                return

            if not self.move_to_goal((0, 0, -math.pi)):
                return

            self.release_block()

            if not self.move_to_goal():
                return

            print(f"Returining to saved pose: {found_pose}")
            if not self.move_to_goal(found_pose):
                return

    def get_camera_tilt(self):
        return self.camera.info['tilt']['command']

    def get_estimated_pose(self):
        if self.use_landmarks:
            return [
                self.estimated_pose.x,
                self.estimated_pose.y,
                self.estimated_pose.theta
            ]
        else:
            return self.base.get_odom()

    def handle_bumper_event(self, event: BumperEvent):
        self.halt = True

    def estimate_block_position(self, bearing_range=None):
        if bearing_range is None:
            bearing_range = self.get_block_bearing_range()

        if bearing_range is None:
            return None

        est_block_x, est_block_y = calc_pos_from_bearing_range(self.get_estimated_pose(), *bearing_range)

        if self.v:
            print(f"Block estimated at {est_block_x}, {est_block_y}")

        return est_block_x, est_block_y


        



if __name__ == "__main__":
    blockbot = BlockBot(True)
    blockbot.execute_sequence()
