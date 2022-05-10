#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan  
class Overtake_Monitor:
    def __init__(self,turtlename):
        # self.scan_sub = rospy.Subscriber(f"{turtlename}/scan", LaserScan, self.scan_callback)
        # self.ovt_pub = rospy.Publisher(f"{turtlename}/overtake", String, queue_size=1)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.ovt_pub = rospy.Publisher("/overtake", String, queue_size=1)
        self.state = "not passing"
        self.count = 0

    # logic goes here, because why not?
    def scan_callback(self, msg):
        print("LENGTHS:")
        # print(len(msg.ranges)) 360
        min1 = min(msg.ranges[0:10])
        min2 = min(msg.ranges[350:359])
        in_front = min([min1, min2])
        back_left = min(msg.ranges[195:200])
        print(self.state)
        print(in_front)
        print(back_left)
        print("+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+=+")

        # state: "passing" or "not passing"
        if self.state == "not passing":
            if in_front < .5:
                self.state = "passing"
                self.ovt_pub.publish(self.state)
        else:
            if (back_left) < .6:
                self.state = "not passing"
                self.ovt_pub.publish(self.state)

if __name__ == '__main__':
    rospy.init_node('overtaker')
    # turtlename = rospy.get_param('~turtlebot')
    turtlename = ""
    ovt = Overtake_Monitor(turtlename)
    rospy.spin()