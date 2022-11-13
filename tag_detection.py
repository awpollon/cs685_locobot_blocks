
import rospy
import math
from apriltag_ros.msg import AprilTagDetectionArray
from interbotix_xs_modules.locobot import InterbotixLocobotXS
from grab_brick import GrabBrick

class TagDetection:
    def __init__(self):
        self.x_move = 0
        self.found_tag = False
        self.align_tag = False
        self.fine_tune = False

    def get_tag_data(self, data):
        filtered_tags = [det for det in data.detections if det.id[0] in [413, 91]]
        if len(filtered_tags) > 0:
            tag = filtered_tags[0]
            #print(tag.pose.pose.pose.position)
            if self.fine_tune:
                print("Finetune")
                if -0.0001 < tag.pose.pose.pose.position.x < 0.04:
                    self.align_tag = True
                    self.x_move = tag.pose.pose.pose.position.y
                    subscription.unregister()
            else:
                if -0.0001 < tag.pose.pose.pose.position.x < 0.04:
                    self.found_tag = True

    def finetune_and_align(self, locobot):
        self.fine_tune = True
        while not rospy.is_shutdown():
            if self.align_tag:
                print("Z-VALUE: ", self.x_move+0.02)
                if self.x_move:
                    GrabBrick(self.x_move)
                else:
                    GrabBrick()
                break
            else:
                locobot.base.move(0, math.pi/10.0, 1)
                rospy.sleep(0.7)


    def rotate_and_find_tag(self):
        locobot = InterbotixLocobotXS("locobot_px100", "mobile_px100") # creates the node too
        global subscription
        subscription = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.get_tag_data)

        num_rots = 0
        while not rospy.is_shutdown():
            if self.found_tag:
                #rospy.signal_shutdown("Found the block")
                print(f"Found the block after {num_rots} rotations")
                self.finetune_and_align(locobot)
                #rospy.signal_shutdown("Found the block")

                break
            else:
#                locobot.base.command_velocity(0, math.pi/6.0)
                locobot.base.move(0, math.pi/8.0, 1)
                rospy.sleep(0.5)
            num_rots += 1

if __name__ == "__main__":
    TagDetection().rotate_and_find_tag()
