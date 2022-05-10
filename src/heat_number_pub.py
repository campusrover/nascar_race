#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32,String


class Heat:
    def __init__(self, string):
        self.ovt_pub = rospy.Publisher("robc/heat",Int32, queue_size=1) 
    
    def pub(self):
        self.ovt_pub.publish(121)


if __name__ == '__main__':
    rospy.init_node('heatp')
    heat = Heat("test")
    while True:
        heat.pub()
        rospy.sleep(1)
    rospy.spin()


    

