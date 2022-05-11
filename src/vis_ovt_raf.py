#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import LaserScan  
from sensor_msgs.msg import CompressedImage
import rospy, cv2, cv_bridge, numpy

from mask_creator import middle_right_moments

class Visual_Overtake_Monitor:
    def __init__(self,turtlename):
        # self.image_sub = rospy.Subscriber(f"{turtlename}/raspicam_node/image/compressed", CompressedImage, self.image_callback)
        # self.ovt_pub = rospy.Publisher(f"{turtlename}/overtake", String, queue_size=1)
        self.image_sub = rospy.Subscriber("rafael/raspicam_node/image/compressed", CompressedImage, self.image_callback)
        self.ovt_pub = rospy.Publisher("rafael/overtake", String, queue_size=1)
        self.ovt_pub_2 = rospy.Publisher("robc/overtake", String, queue_size=1)
        self.start_sub = rospy.Subscriber("rafael/start", Int32, self.start_callback)
        self.raf_passing = False
        self.robc_passing = False
        self.count = 0
        self.image = None
        self.started = False

    def start_callback(self, msg):
        if msg.data == 0:
            self.started = True
    
    def image_callback(self, msg):
        self.image = msg

    def image_process(self):
        if not self.started:
            return
        result = middle_right_moments(self.image)
        if result[0] > 1000000: # 1 million is close enough for us
            if self.raf_passing:
                self.raf_passing = True
                self.ovt_pub.publish("passing")
        
        if result[1] > 600000:
            # tell the OTHER robot to come BACK to center
            if self.robc_passing:
                self.robc_passing = False
                self.ovt_pub_2.publish("not passing")
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('overtaker')
    # turtlename = rospy.get_param('~turtlebot')
    ovt = Visual_Overtake_Monitor("asdsad")
    rospy.sleep(3)
    while not rospy.is_shutdown():
        ovt.image_process()
        rospy.Rate(1)
    rospy.spin()