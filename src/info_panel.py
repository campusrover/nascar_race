#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32,String,Float32


class InfoPanel:
    def __init__(self,turtlename):
        
        self.bridge = cv_bridge.CvBridge()
        self.lap_sub = rospy.Subscriber(f"{turtlename}/lapCount", Int32, self.lap_callback)
        self.start_sub = rospy.Subscriber(f"{turtlename}/start", Int32, self.start_callback)
        self.heat_sub = rospy.Subscriber(f"{turtlename}/heat", Float32, self.heat_callback)
        self.ovh_sub = rospy.Subscriber(f"{turtlename}/overheat", String, self.ovh_callback)
        self.speed_sub = rospy.Subscriber(f"{turtlename}/speed", String, self.speed_cb)
        self.started = False
        self.heat = 0.0
        self.lap_count = 0
        self.countdown = 5
        self.overheated = False
        self.time = 0.0
        self.speed = 0.0

    def start_callback(self, msg):
        self.countdown = msg.data
        if msg.data == 0:
            self.started = True
            self.time = rospy.get_time()

    def lap_callback(self, msg):
        self.lap_count = msg.data

    def heat_callback(self, msg):
        self.heat = msg.data

    def speed_cb(self, msg):
        self.speed = msg.data
    
    def ovh_callback(self,msg):
        if msg.data == "no":
            self.overheated = False
        else:
            self.overheated = True

    def update(self):
        # for i in range (0,30):
        #     print()
        s = f"{chr(10)*23}"
        print(s)
        print("=================")
        if not self.started:
            print(self.countdown)
        else:
            print("Lap: " + str(self.lap_count))
            print("Speed: " + str(self.speed * 10))
            if self.overheated:
                print("Overheated!!: " + str(self.heat)[:4])
            else:
                print("Car Heat: " + str(self.heat)[:4] + " (Max 100)")
        print("=================")

if __name__ == '__main__':
    rospy.init_node("panel")
    turtlename = rospy.get_param('~turtlebot')
    info = InfoPanel(turtlename)
    while not rospy.is_shutdown():
        info.update()
        rospy.Rate(1).sleep()
    rospy.spin()

