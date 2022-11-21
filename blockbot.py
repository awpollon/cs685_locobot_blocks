import math
import rospy
import numpy as np
from enum import Enum
from apriltag_ros.msg import AprilTagDetectionArray
from interbotix_xs_modules.locobot import InterbotixLocobotXS
from localizer import BlockBotLocalizer, calc_pos_from_bearing_range, calc_bearing_range_from_tag
from locobot_controller import LocobotController


class RobotActionState(Enum):
    WAIT = 0
    SEARCH_FOR_BLOCK = 1
    TRAVEL_TO_BLOCK = 2
    ALIGN_WITH_BLOCK = 3
    PICK_UP_BLOCK = 4
    MOVE_TO_GOAL = 5
    RELEASE_BLOCK = 6
    RETURN_HOME = 7


BLOCK_TAGS = [91, 685]
LANDMARK_TAGS = [680, 681, 682, 683, 684, 86]
BIN_TAG = 413
TAGS = [*BLOCK_TAGS, *LANDMARK_TAGS, BIN_TAG]

ROTATION_INCREMENT = math.pi/20.0
MOVE_INCREMENT = 0.05

CAMERA_SETTINGS = {"tilt": 1, "search_tilt": 4*math.pi/16, "pan": 0, "height": 0.45}

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
    theta_rel = theta_rel % (2 * np.pi)

    if theta_rel > math.pi:
        theta_rel -= 2 * np.pi
        
    return theta_rel


class BlockBot(InterbotixLocobotXS):
    def __init__(self, align_camera=True) -> None:
        super().__init__("locobot_px100", "mobile_px100")
        rospy.Subscriber("/tag_detections",
                         AprilTagDetectionArray, self.get_tag_data)

        self.tags_data = []
        self.block_tag_data = None
        self.bin_tag_data = None
        self.found_block = False
        self.controller = LocobotController()

        self.action_state = RobotActionState.WAIT
        self.initialize_robot(align_camera)

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
        camera_tilt = self.get_camera_tilt()
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
            self.block_tag_data = block_tag[0].pose.pose.pose.position
        # else:
        #     self.block_position = None

        bin_tag = [tag for tag in data.detections if tag.id[0] == BIN_TAG]
        if len(bin_tag) > 0:
            self.bin_tag_data = bin_tag[0].pose.pose.pose.position
        else:
            self.bin_tag_data = None

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
        self.block_tag_data = None
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
        self.camera.move("tilt", CAMERA_SETTINGS["search_tilt"])
        for _ in range(500):
            if type == "block":
                pos = self.block_tag_data
            elif type == "bin":
                pos = self.bin_tag_data

            if not pos:  # No block in view, keep searching
                self.move(0, 2 * ROTATION_INCREMENT, 0.5)
            else:  # Block in view
                if self.action_state == RobotActionState.SEARCH_FOR_BLOCK:
                    # ONLY RUNS ONCE IN LOOP

                    # Estimate block location and travel to it
                    self.action_state = RobotActionState.TRAVEL_TO_BLOCK

                    camera_tilt = self.get_camera_tilt()
                    block_bearing, block_range = calc_bearing_range_from_tag(pos, camera_tilt)

                    est_block_x, est_block_y, _ = calc_pos_from_bearing_range(self.get_estimated_pose(), block_bearing, block_range)
                    
                    print(f"Block estimated at {est_block_x}, {est_block_y}")

                    stop_dist = .17
                    dx = stop_dist * np.cos(block_bearing)
                    dy = stop_dist * np.sin(block_bearing)

                    target_pose = (est_block_x - dx, est_block_y - dy, block_bearing)
                    if not self.move_to_goal(target_pose):
                        print("Unable to reach block")
                        self.action_state = RobotActionState.WAIT
                        return

                    # Near block, now align
                    self.camera.move("tilt", CAMERA_SETTINGS["tilt"])
                    self.action_state = RobotActionState.ALIGN_WITH_BLOCK

                if self.action_state == RobotActionState.ALIGN_WITH_BLOCK:
                    if GRABBABLE_MARGIN[0] < pos.x < GRABBABLE_MARGIN[1]: # Block in grabbable margin
                        if abs(pos.z - GRABBABLE_APRILTAG_Z) > 0.01:  # Block is far away                        
                            self.move(MOVE_INCREMENT, 0, 0.5)
                        else:  # Block is grabbable
                            return True
                    else:  # Block not in grabbable margin
                        self.move(
                            0, ROTATION_INCREMENT if pos.x < 0 else -ROTATION_INCREMENT, 0.5)
        
        print("Block search limit reached")
        self.action_state = RobotActionState.WAIT

    def move_to_goal(self, goal_pose=(0, 0, 0)):
        print(f"Starting move to goal: {goal_pose}")
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
                self.action_state == RobotActionState.RETURN_HOME
                # TODO: Could go back to where block was found to continue search
                self.move_to_goal()

    def get_camera_tilt(self):
        return -self.camera.info['tilt']['command']

    def get_estimated_pose(self):
        return (self.estimated_pose.x(), self.estimated_pose.y(), self.estimated_pose.theta())


if __name__ == "__main__":
    blockbot = BlockBot(True)
    blockbot.execute_sequence()
