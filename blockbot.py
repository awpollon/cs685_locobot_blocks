import math
import rospy
from enum import Enum
from apriltag_ros.msg import AprilTagDetectionArray
from interbotix_xs_modules.locobot import InterbotixLocobotXS
from localizer import BlockBotLocalizer


class RobotActionState(Enum):
    WAIT = 0
    SEARCH_FOR_BLOCK = 1
    PICK_UP_BLOCK = 2
    TRAVEL_TO_GOAL = 3
    MOVE_FORWARD = 4


BLOCK_TAGS = [91, 685]
LANDMARK_TAGS = [680, 681, 682, 683, 684]
BIN_TAG = 413
TAGS = [*BLOCK_TAGS, *LANDMARK_TAGS, BIN_TAG]

ROTATION_INCREMENT = math.pi/20.0
MOVE_INCREMENT = 0.03

CAMERA_SETTINGS = {"tilt": 1, "pan": 0, "height": 0.45}

GRABBABLE_APRILTAG_Z = 0.5
GRABBABLE_MARGIN = [-0.01, 0.01]


class BlockBot(InterbotixLocobotXS):
    def __init__(self) -> None:
        super().__init__("locobot_px100", "mobile_px100")
        rospy.Subscriber("/tag_detections",
                         AprilTagDetectionArray, self.get_tag_data)

        self.tags_data = []
        self.block_position = None
        self.found_block = False
        self.action_state - RobotActionState.WAIT
        self.initialize_robot()

    def initialize_robot(self):
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
        # Process odometry
        odom = self.base.get_odom()
        landmarks = [
            tag for tag in self.tags_data if tag.id[0] in LANDMARK_TAGS]

        self.localizer.add_observation(odom, landmarks)
        self.localizer.optmize()
        self.localizer.estimated_pose

        print("Estimated pose: ")
        print(self.localizer.estimated_pose)
        print("Covariance:")
        print(self.localizer.current_covariance)
        print("Odometry measurement")
        print(odom)

    def get_tag_data(self, data):
        self.tags_data = [tag for tag in data.detections if tag.id[0] in TAGS]
        block_tag = [tag for tag in data.detections if tag.id[0] in BLOCK_TAGS]
        if len(block_tag) > 0:
            self.block_position = block_tag[0].pose.pose.pose.position
        else:
            self.block_position = None

    def grab_block(self):
        self.action_state = RobotActionState.PICK_UP_BLOCK
        self.base.move(0, 2*ROTATION_INCREMENT if self.block_position.x <
                       0 else 2*ROTATION_INCREMENT, 1)  # TODO: refactor
        self.arm.go_to_home_pose()
        self.arm.set_ee_cartesian_trajectory(z=-0.25)
        self.gripper.close()
        self.arm.go_to_home_pose()
        self.base.reset_odom()
        rospy.sleep(2.5)
        self.gripper.open()
        self.arm.go_to_sleep_pose()
        self.action_state = RobotActionState.WAIT

    def find_block(self):
        self.action_state = RobotActionState.SEARCH_FOR_BLOCK
        while not rospy.is_shutdown():
            if not self.block_position:  # No block in view
                print("Searching...")
                self.base.move(0, 2 * ROTATION_INCREMENT, 0.5)
            else:  # Block in view
                # Block in grabbable margin
                if GRABBABLE_MARGIN[0] < self.block_position.x < GRABBABLE_MARGIN[1]:
                    if abs(self.block_position.z - GRABBABLE_APRILTAG_Z) > 0.01:  # Block is far away
                        print("Aligned. Moving...",
                              self.block_position.z, self.block_position.x)
                        self.base.move(MOVE_INCREMENT, 0, 0.5)
                    else:  # Block is grabbable
                        print("\n\nPicking...", self.block_position)
                        self.grab_block()
                        rospy.signal_shutdown("Found the block")
                else:  # Block not in grabbable margin
                    print("Found. Aligning...", self.block_position.x)
                    self.base.move(
                        0, ROTATION_INCREMENT if self.block_position.x < 0 else -ROTATION_INCREMENT, 0.5)
        self.action_state = RobotActionState.WAIT

    def execute_sequence(self):
        self.find_block()


if __name__ == "__main__":
    blockbot = BlockBot()
    blockbot.execute_sequence()
