#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan  
from sensor_msgs.msg import CompressedImage
import rospy, cv2, cv_bridge, numpy

from mask_creator import process_all_the_way_pink_middle, process_all_the_way_yellow_right

class Visual_Overtake_Monitor:
    def __init__(self,turtlename):
        # self.image_sub = rospy.Subscriber(f"{turtlename}/raspicam_node/image/compressed", CompressedImage, self.image_callback)
        # self.ovt_pub = rospy.Publisher(f"{turtlename}/overtake", String, queue_size=1)
        self.image_sub = rospy.Subscriber("rafael/raspicam_node/image/compressed", CompressedImage, self.image_callback)
        self.ovt_pub = rospy.Publisher("rafael/overtake", String, queue_size=1)
        self.image_sub_2 = rospy.Subscriber("robc/raspicam_node/image/compressed", CompressedImage, self.image_callback_2)

        self.state = "not passing"
        self.count = 0
        self.image = None
        self.image_2 = None
    
    def image_callback(self, msg):
        self.image = msg

    def image_callback_2(self, msg):
        self.image_2 = msg

    def image_process(self):
        moments_middle = process_all_the_way_pink_middle(self.image)

        if moments_middle['m00'] > 1000000: # 1 million is close enough for us
            if self.state == "not passing":
                self.state = "passing"
                self.ovt_pub.publish("passing")
        # cv2.imshow("image", image)
        cv2.waitKey(3)
    
    def image_process_2(self):
        moments_right = process_all_the_way_yellow_right(self.image_2)

        # if this second cam sees the second color, then it should pull it back
        if moments_right['m00'] > 600000: # 1 million is close enough for us
            if self.state == "passing":
                self.state = "not passing"
                self.ovt_pub.publish("not passing")
        # cv2.imshow("image", image)
        cv2.waitKey(3)


if __name__ == '__main__':
    rospy.init_node('overtaker')
    # turtlename = rospy.get_param('~turtlebot')
    ovt = Visual_Overtake_Monitor("asdsad")
    rospy.sleep(3)
    while not rospy.is_shutdown():
        ovt.image_process()
        rospy.Rate(0.5)
    rospy.spin()