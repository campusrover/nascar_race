#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32,String


class ColorThing:
    def __init__(self,turtlename):
        
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber(f"{turtlename}/raspicam_node/image/compressed", CompressedImage, self.image_callback)
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
        lower_green = numpy.array([305/2,65*2.55,20*2.55])
        upper_green = numpy.array([335/2,80*2.55,50*2.55]) # dark pink square hsv 

        # lower_green = numpy.array([70/2,50*2.55,55*2.55])
        # upper_green = numpy.array([80/2,80*2.55,80*2.55]) # yellow square hsv 

        mask = cv2.inRange(hsv,  lower_green, upper_green)
        masked = cv2.bitwise_and(image, image, mask=mask)
        # ignore all of image except 30 pixel slice at bottom
        h, w, d = image.shape
        
        #CROPPER
        #HEIGHT show only the top of the mask (everything below first 1/3 is hidden)
        search_top = int(h /3)
        mask[search_top:h, 0:w] = 0

        #WIDTH (MIDDLE) show only the middle of the mask (hide the left 1/3, and right -1/3)
        search_left = int(w/3)
        search_right = int(2*w/3)
        mask[0:h, 0:search_left] = 0
        mask[0:h, search_right:w] = 0

        #WIDTH (RIGHT) show only the top right 1/3 corner of the mask
        # search_left = int(2*w/3)
        # mask[0:h, 0:search_left] = 0

        #show mask
        cv2.imshow("robovision", mask)

        # Lap Counter
        M = cv2.moments(mask)
        self.logcount += 1
        # if a color is detected, M['m00'] will return a nonzero value
        if M['m00'] > 0:
            print(M['m00'])
        cv2.imshow("image", image)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('colorthing')
    lapcounter = ColorThing("rafael")
    rospy.sleep(3) # to allow the camera to initialize PLUS the time to start the race
    while not rospy.is_shutdown():
        lapcounter.image_process()  
        rospy.Rate(3)
    rospy.spin()

