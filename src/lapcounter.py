#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32,String


class LapCounter:
    def __init__(self,turtlename):
        
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(f"{turtlename}/raspicam_node/image/compressed", CompressedImage, self.image_callback)
        self.lap_pub = rospy.Publisher(f"{turtlename}/lapCount",Int32,queue_size=1)
        self.ovt_pub = rospy.Publisher(f"{turtlename}/overtake",String, queue_size=1) 
        self.twist = Twist()
        self.logcount = 0
        self.lostcount = 0
        self.lapcount = 0
        self.time_since_detect = 0
        self.image = None

    def image_callback(self, msg):
        self.image = msg

    def image_process(self):
        # get image from camera
        image = self.bridge.compressed_imgmsg_to_cv2(self.image, desired_encoding = "bgr8")
        # convert bgr8 image to hsv
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # hsv value arrays
        lower_green = numpy.array([30,180,100])
        upper_green = numpy.array([90,255,200]) # neon green hsv 

        mask = cv2.inRange(hsv,  lower_green, upper_green)
        masked = cv2.bitwise_and(image, image, mask=mask)

        # ignore all of image except 30 pixel slice at bottom
        h, w, d = image.shape
        search_top = int(3 * h /4)
        search_bot = search_top +10
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0

        # Lap Counter
        M = cv2.moments(mask)
        self.logcount += 1
        # if a color is detected, M['m00'] will return a nonzero value
        if M['m00'] > 0 and rospy.get_time() >= self.time_since_detect+5:
            self.lapcount+=1
            self.lap_pub.publish(self.lapcount)
            self.time_since_detect = rospy.get_time()
            
            # self.ovt_pub.publish("passing") # UNCOMMENT THIS TO TEST OVERTAKE
            # ADD COOLDOWN
        #print(self.lapcount)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('lapcounter')
    turtlename = rospy.get_param('~turtlebot')
    lapcounter = LapCounter(turtlename)
    rospy.sleep(10) # to allow the camera to initialize
    while not rospy.is_shutdown():
        lapcounter.image_process()  
        rospy.Rate(0.5)
    rospy.spin()

