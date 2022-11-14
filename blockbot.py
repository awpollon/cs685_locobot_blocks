import math
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from interbotix_xs_modules.locobot import InterbotixLocobotXS

BLOCK_TAG = 91
LANDMARK_TAG = 86
BIN_TAG = 413

TAGS = [BLOCK_TAG, LANDMARK_TAG, BIN_TAG]


class BlockBot(InterbotixLocobotXS):
    def initialize_robot(self):
        self.found_block = False
        self.block_position = None
        self.tags_data = []

        self.camera.move("pan", 0)
        self.camera.move("tilt", 1)
        self.arm.go_to_sleep_pose()
        self.gripper.close()
        self.gripper.open()
        self.base.reset_odom()

    def get_tag_data(self, data):
        self.tags_data = [tag for tag in data.detections if tag.id[0] in TAGS]
        self.search_block()

    def search_block(self):
        tags = [tag for tag in self.tags_data if tag.id[0] == BLOCK_TAG]
        if len(tags) > 0 and -0.01 < tags[0].pose.pose.pose.position.x < 0.01:
            self.found_block = True
            self.block_position = tags[0].pose.pose.pose.position


    def grab_block(self):
        base_x = 0
        if False and abs(self.block_position.z - 4.9) > 1:
            base_x = 2 * abs(self.block_position.z - 4.9)
        self.base.move(base_x, math.pi/20, 1)
        self.arm.go_to_home_pose()
        self.arm.set_ee_cartesian_trajectory(z=-0.25)
        self.gripper.close()
        self.arm.go_to_home_pose()
        self.gripper.open()
        self.arm.go_to_sleep_pose()
#        self.base.move(-base_x, 0, 1)

    def rotate_and_find_tag(self):
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.get_tag_data)

        num_rots = 0
        while not rospy.is_shutdown():
            if self.found_block:
                print(f"Found the block after {num_rots} rotations")
                self.grab_block()
                rospy.signal_shutdown("Found the block")
                break
            elif num_rots > 0:
                self.base.move(0, math.pi/18.0, 0.5)
            num_rots += 1

    def execute_sequence(self):
        blockbot.initialize_robot()
        self.rotate_and_find_tag()


if __name__ == "__main__":
    blockbot = BlockBot("locobot_px100", "mobile_px100")
    blockbot.execute_sequence()
