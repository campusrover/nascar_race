#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import CompressedImage

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed', CompressedImage, self.image_callback)
        
        #https://docs.opencv.org/4.x/df/d9d/tutorial_py_colorspaces.html FINDING HSV VALUES USING BGR (BLUE GREEN RED)
        #BGR --> HSV
        bgr = numpy.uint8([[[200,200,200 ]]]) #input BGR here
        to_hsv = cv2.cvtColor(bgr,cv2.COLOR_BGR2HSV)
        print("HSV VALUE: ", to_hsv)
        #HSV --> BGR
        #hsv = numpy.uint8([[[120,255,255]]]) #input HSV here
        #to_bgr = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
        #print("BGR VALUE: ", to_bgr)

    def image_callback(self, msg):
        image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding = "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # [0-180, 0-255, 0-255]
        
        # lower_yellow = numpy.array([10,20,110])
        # upper_yellow = numpy.array([40,80,150]) # single layer blu tape under the stairs

        lower_arr = [80/2,0*2.55,70*2.55]
        upper_arr = [180/2,25*2.55,100*2.55]

        lower_yellow = numpy.array(lower_arr)
        upper_yellow = numpy.array(upper_arr) # masking tape, far test bench (bright line, red and gray carpet)


        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        masked = cv2.bitwise_and(image, image, mask=mask)
        cv2.imshow("mask", mask)
        cv2.imshow("masked", masked)
        cv2.imshow("original", image)
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()

#https://stackoverflow.com/questions/10948589/choosing-the-correct-upper-and-lower-hsv-boundaries-for-color-detection-withcv
#https://stackoverflow.com/questions/22588146/tracking-white-color-using-python-opencv WHITE COLOR TRACKING

#Masking Tape Color:
#https://color-hex.org/color/fcf7dd

#cv2.transpose
