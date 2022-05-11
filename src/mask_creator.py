import rospy
from std_msgs.msg import String, Int32
from sensor_msgs.msg import LaserScan  
from sensor_msgs.msg import CompressedImage
import rospy, cv2, cv_bridge, numpy


def middle_right_moments(input):
    bridge = cv_bridge.CvBridge()
    # get image from camera
    image = bridge.compressed_imgmsg_to_cv2(input, desired_encoding = "bgr8")

    # convert bgr8 image to hsv
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # hsv value arrays
    lower_green = numpy.array([305/2,65*2.55,20*2.55])
    upper_green = numpy.array([335/2,80*2.55,50*2.55]) # dark pink square hsv 

    mask = cv2.inRange(hsv,  lower_green, upper_green)
    masked = cv2.bitwise_and(image, image, mask=mask)
    # ignore all of image except 30 pixel slice at bottom
    h, w, d = image.shape

    middle = mask.copy()
    right = mask.copy()
    
    #CROPPER
    #HEIGHT show only the top of the mask (everything below first 1/3 is hidden)
    search_top = int(h/3)
    middle[search_top:h, 0:w] = 0

    #WIDTH (MIDDLE) show only the middle of the mask (hide the left 1/3, and right -1/3)
    search_left = int(w/3)
    search_right = int(2*w/3)
    middle[0:h, 0:search_left] = 0
    middle[0:h, search_right:w] = 0

    moments_middle = cv2.moments(middle)

    #WIDTH (RIGHT) show only the top right 1/3 corner of the mask
    search_left2 = int(2*w/3)
    right[0:h, 0:search_left2] = 0

    #show mask
    cv2.imshow("robovision", right)

    moments_right = cv2.moments(right)

    return [moments_middle['m00'], moments_right['m00']]