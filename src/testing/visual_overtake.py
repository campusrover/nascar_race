#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan  
class Visual_Overtake_Monitor:
    def __init__(self,turtlename):
        # self.image_sub = rospy.Subscriber(f"{turtlename}/raspicam_node/image/compressed", CompressedImage, self.image_callback)
        # self.ovt_pub = rospy.Publisher(f"{turtlename}/overtake", String, queue_size=1)
        self.image_sub = rospy.Subscriber(f"{turtlename}/raspicam_node/image/compressed", CompressedImage, self.image_callback)
        self.ovt_pub = rospy.Publisher(f"{turtlename}/overtake", String, queue_size=1)
        self.robot_state = "not passing"
        self.count = 0
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

        mask = cv2.inRange(hsv,  lower_green, upper_green)
        masked = cv2.bitwise_and(image, image, mask=mask)
        # ignore all of image except 30 pixel slice at bottom
        h, w, d = image.shape

        mask_middle = mask
        mask_right = mask
        
        #CROPPER
        #HEIGHT show only the top of the mask (everything below first 1/3 is hidden)
        search_top = int(h /3)
        mask_middle[search_top:h, 0:w] = 0
        mask_right[search_top:h, 0:w] = 0

        #WIDTH (MIDDLE) show only the middle of the mask (hide the left 1/3, and right -1/3)
        search_left = int(w/3)
        search_right = int(2*w/3)
        mask_middle[0:h, 0:search_left] = 0
        mask_middle[0:h, search_right:w] = 0

        #WIDTH (RIGHT) show only the top right 1/3 corner of the mask
        search_left = int(2*w/3)
        mask_right[0:h, 0:search_left] = 0

        #show mask
        cv2.imshow("robovision", mask_middle)
        cv2.imshow("robovision_2", mask_right)

        # Lap Counter
        M = cv2.moments(mask)
        moments_middle = cv2.moments(mask_middle)
        moments_right = cv2.moments(mask_right)

        # if a color is detected, M['m00'] will return a nonzero value
        if M['m00'] > 0:
            print(M['m00'])

        if moments_middle['m00'] > 1000000: # 1 million is close enough for us
            if self.state == "not passing":
                self.state = "passing"
                self.ovt_pub.publish("passing")
        cv2.imshow("image", image)
        cv2.waitKey(3)


if __name__ == '__main__':
    rospy.init_node('overtaker')
    turtlename = rospy.get_param('~turtlebot')
    ovt = Visual_Overtake_Monitor(turtlename)
    while not rospy.is_shutdown():
        ovt.image_process()
        rospy.Rate(3)
    rospy.spin()