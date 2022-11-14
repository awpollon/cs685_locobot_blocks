import math
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from interbotix_xs_modules.locobot import InterbotixLocobotXS


class BlockBot(InterbotixLocobotXS):
    def initialize_robot(self):
        self.found_tag = False
        self.position = None
        self.camera.move("pan", 0)
        self.camera.move("tilt", 1)
        self.arm.go_to_sleep_pose()
        self.gripper.close()
        self.gripper.open()
        self.base.reset_odom()

    def get_tag_data(self, data):
        filtered_tags = [det for det in data.detections if det.id[0] in [413, 91]]
        if len(filtered_tags) > 0:
            tag = filtered_tags[0]
            if -0.01 < tag.pose.pose.pose.position.x < 0.01:
                self.found_tag = True
                self.position = tag.pose.pose.pose.position
                print(tag.pose.pose.pose.position)

    def grab_block(self):
        base_x = 0
        if abs(self.position.z - 4.9) > 1:
            base_x = 2 * abs(self.position.z - 4.9)
        self.base.move(base_x, math.pi/20, 1)
        self.arm.go_to_home_pose()
        self.arm.set_ee_cartesian_trajectory(z=-0.25)
        self.gripper.close()
        self.arm.go_to_home_pose()
        self.gripper.open()
        self.arm.go_to_sleep_pose()
        self.base.move(-base_x, 0, 1)

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
                self.base.move(0, math.pi/18.0, 0.5)
            num_rots += 1

    def execute_sequence(self):
        blockbot.initialize_robot()
        self.rotate_and_find_tag()


if __name__ == "__main__":
    blockbot = BlockBot("locobot_px100", "mobile_px100")
    blockbot.execute_sequence()
