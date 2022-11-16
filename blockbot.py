import math
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from interbotix_xs_modules.locobot import InterbotixLocobotXS

BLOCK_TAGS = [91, 685]
LANDMARK_TAG = 86
BIN_TAG = 413
TAGS = [*BLOCK_TAGS, LANDMARK_TAG, BIN_TAG]

ROTATION_ANGLE = math.pi/20.0
CAMERA_SETTINGS = {"tilt": 1, "pan": 0, "height": 0.45}
GRABBABLE_APRILTAG_Z = 0.51
GRABBABLE_MARGIN = [-0.01, 0.01]
GROUND_INCREMENT = 0.01
ITERATION_LIMIT = 30


class BlockBot(InterbotixLocobotXS):
    def __init__(self) -> None:
        super().__init__("locobot_px100", "mobile_px100")        
        self.found_block = False
        self.block_position = None
        self.base.reset_odom()

        # Track tag detections
        self.tags_data = []
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.get_tag_data)

        self.camera.move("pan", CAMERA_SETTINGS["pan"])
        self.camera.move("tilt", CAMERA_SETTINGS["tilt"])
        self.arm.go_to_sleep_pose()
        self.gripper.close()
        self.gripper.open()

    def get_tag_data(self, data):
        self.tags_data = [tag for tag in data.detections if tag.id[0] in TAGS]
        self.search_block()

    def search_block(self):
        tags = [tag for tag in self.tags_data if tag.id[0] in BLOCK_TAGS]
        if len(tags) > 0 and GRABBABLE_MARGIN[0] < tags[0].pose.pose.pose.position.x < GRABBABLE_MARGIN[1]:
            self.found_block = True
            self.block_position = tags[0].pose.pose.pose.position

    def grab_block(self):
#        self.base.move(0, ROTATION_ANGLE, 1)
        dist_moved = 0
        iter = 0
        while iter < ITERATION_LIMIT and abs(self.block_position.z - GRABBABLE_APRILTAG_Z) > 0.01: # move 1 cm at a time until the  block is grabbable
            print("Moving", iter, self.block_position.z, abs(self.block_position.z - GRABBABLE_APRILTAG_Z))
            self.base.move(GROUND_INCREMENT, 0, 1)
            dist_moved += GROUND_INCREMENT
            iter += 1
            print([t for t in self.tags_data if t.id[0] in TAGS])
        self.arm.go_to_home_pose()
        self.arm.set_ee_cartesian_trajectory(z=-0.25)
        self.gripper.close()
        self.arm.go_to_home_pose()
        self.gripper.open()
        self.arm.go_to_sleep_pose()
#        if distance_moved > 0:
#            self.base.move(-distance_moved, 0, 1)

    def get_ground_distance(april_tag_z):
        return math.sqrt(april_tag_z**2 - CAMERA_SETTINGS['height']**2)

    def rotate_and_find_tag(self):
        num_rots = 0
        while not rospy.is_shutdown() and num_rots < ITERATION_LIMIT:
            if self.found_block:
                print(f"Found the block after {num_rots} rotations")
                self.grab_block()
                rospy.signal_shutdown("Found the block")
                break
            elif num_rots > 0:
                self.base.move(0, ROTATION_ANGLE, 0.5)
            num_rots += 1

    def execute_sequence(self):
        blockbot.initialize_robot()
        self.rotate_and_find_tag()


if __name__ == "__main__":
    blockbot = BlockBot()
    blockbot.execute_sequence()
