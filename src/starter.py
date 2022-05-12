#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32,String


class Starter:
    def __init__(self, turtlename):
        self.start_pub = rospy.Publisher("rafael/start",Int32, queue_size=1) 
        self.start_pub_2 = rospy.Publisher("robc/start",Int32, queue_size=1) 
    
    def pub(self, num):
        self.start_pub.publish(num)
        self.start_pub_2.publish(num)

if __name__ == '__main__':
    rospy.init_node('starter')
    starter = Starter("x")
    for i in range (0,6):
        cd = 5-i
        starter.pub(cd)
        # print(cd)
        rospy.sleep(1)

    rospy.spin()


    

