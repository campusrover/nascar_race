#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan  
class Overtake_Monitor:
    def __init__(self,turtlename):
        self.scan_sub = rospy.Subscriber(f"{turtlename}/scan", LaserScan, self.scan_callback)
        self.ovt_pub = rospy.Publisher(f"{turtlename}/overtake", String, queue_size=1)
        self.state = "not passing"
        self.count = 0

    # logic goes here, because why not?
    def scan_callback(self, msg):
        # if(len(msg.ranges)<350):
        #     # print(len(msg.ranges))
        #     print(msg.ranges[240:245])
        #     return
        print("LENGTHS:")
        print(len(msg.ranges)) 
        # print(msg.ranges[350:359])
        
        min_front = 999.0
        min_back = 999.0
        for i in range (0,len(msg.ranges)):
            if 0 < i < 10:
                if (msg.ranges[i] < min_front) and (msg.ranges[i] > 0.0):
                    min_front = msg.ranges[i]
            if 240 < i < 245:
                if (msg.ranges[i] < min_back) and (msg.ranges[i] > 0.0):
                    min_back = msg.ranges[i]

        # min1 = min(msg.ranges[0:10])
        # min2 = min(msg.ranges[240:245])

        in_front = min([min_front, min_back])
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
        # else:
        #     if (back_left) < .6:
        #         self.state = "not passing"
        #         self.ovt_pub.publish(self.state)

if __name__ == '__main__':
    rospy.init_node('overtaker')
    turtlename = rospy.get_param('~turtlebot')
    ovt = Overtake_Monitor(turtlename)
    rospy.spin()