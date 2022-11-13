import math
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from interbotix_xs_modules.locobot import InterbotixLocobotXS

class BlockBot:
    def __init__(self):
        self.found_tag = False
        self.initialize_robot()

    def initialize_robot(self):
        self.locobot = InterbotixLocobotXS("locobot_px100", "mobile_px100")
        self.locobot.camera.move("pan", 0)
        self.locobot.camera.move("tilt", 1)
        self.locobot.arm.go_to_sleep_pose()
        self.locobot.gripper.close()
        self.locobot.gripper.open()

    def get_tag_data(self, data):
        filtered_tags = [det for det in data.detections if det.id[0] in [413, 91]]
        if len(filtered_tags) > 0:
            tag = filtered_tags[0]
            if -0.01 < tag.pose.pose.pose.position.x < 0.01:
                self.found_tag = True
                print(tag.pose.pose.pose.position)

    def grab_block(self):
        self.locobot.base.move(0, math.pi/20, 0.5)
        self.locobot.arm.go_to_home_pose()
        self.locobot.arm.set_ee_cartesian_trajectory(z=-0.25)
        self.locobot.gripper.close()
        self.locobot.arm.go_to_home_pose()
        self.locobot.gripper.open()
        self.locobot.arm.go_to_sleep_pose()

    def rotate_and_find_tag(self):
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.get_tag_data)

        num_rots = 0
        while not rospy.is_shutdown():
            if self.found_tag:
                print(f"Found the block after {num_rots} rotations")
                self.grab_block()
                rospy.signal_shutdown("Found the block")
                break
            elif num_rots > 0:
                self.locobot.base.move(0, math.pi/20.0, 0.5)
            num_rots += 1

    def execute_sequence(self):
        self.rotate_and_find_tag()

if __name__ == "__main__":
    blockbot = BlockBot()
    blockbot.execute_sequence()
