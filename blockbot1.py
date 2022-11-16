import math
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from interbotix_xs_modules.locobot import InterbotixLocobotXS

BLOCK_TAGS = [91, 685]
LANDMARK_TAG = 86
BIN_TAG = 413
TAGS = [*BLOCK_TAGS, LANDMARK_TAG, BIN_TAG]

ROTATION_INCREMENT = math.pi/20.0
MOVE_INCREMENT = 0.02

CAMERA_SETTINGS = {"tilt": 1, "pan": 0, "height": 0.45}

GRABBABLE_APRILTAG_Z = 0.51
GRABBABLE_MARGIN = [-0.01, 0.01]

ITERATION_LIMIT = 50

class BlockBot(InterbotixLocobotXS):
    def initialize_robot(self):
        self.tags_data = []
        self.block_position = None
        self.found_block = False

        self.camera.move("pan", CAMERA_SETTINGS["pan"])
        self.camera.move("tilt", CAMERA_SETTINGS["tilt"])
        self.arm.go_to_sleep_pose()
        self.base.reset_odom()

        # fix any erroneous gripper positions
        self.gripper.close()
        self.gripper.open()

    def get_tag_data(self, data):
        self.tags_data = [tag for tag in data.detections if tag.id[0] in TAGS]
        block_tag = [tag for tag in data.detections if tag.id[0] in BLOCK_TAGS]
        if len(block_tag) > 0:
            self.block_position = block_tag[0].pose.pose.pose.position
        else:
            self.block_position = None

    # TODO: Remove
    def grab_block_old(self):
        dist_moved = 0
        while abs(self.block_position.z - GRABBABLE_APRILTAG_Z) > 0.01: # move 1 cm at a time until the  block is grabbable
            self.base.move(GROUND_INCREMENT, 0, 1)
            dist_moved += GROUND_INCREMENT
            iter += 1

    def grab_block(self):
        self.arm.go_to_home_pose()
        self.arm.set_ee_cartesian_trajectory(z=-0.25)
        self.gripper.close()
        self.arm.go_to_home_pose()
        self.gripper.open()
        self.arm.go_to_sleep_pose()

    def find_block(self):
        while not rospy.is_shutdown():
            if not self.block_position: # No block in view
                print("Searching...")
                self.base.move(0, 2 * ROTATION_INCREMENT, 0.5)
            else: # Block in view
                if GRABBABLE_MARGIN[0] < self.block_position.x < GRABBABLE_MARGIN[1]: # Block in grabbable margin
                    if abs(self.block_position.z - GRABBABLE_APRILTAG_Z) > 0.01:
                    
                else: # Block not in grabbable margin
                    print("Found. Aligning...", self.block_position.x)
                    self.base.move(0, ROTATION_INCREMENT, 0.5)
        # rospy.signal_shutdown("Found the block")

    def execute_sequence(self):
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.get_tag_data)
        blockbot.initialize_robot()
        self.find_block()


if __name__ == "__main__":
    blockbot = BlockBot("locobot_px100", "mobile_px100")
    blockbot.execute_sequence()
