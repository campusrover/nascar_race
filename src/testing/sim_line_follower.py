#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String,Int32,Float32

class Follower:
    def __init__(self,turtlename):  
        self.bridge = cv_bridge.CvBridge()
        self.speed_sub = rospy.Subscriber("/speed", Int32, self.speed_callback)
        self.image_sub = rospy.Subscriber("/camera/image/compressed", CompressedImage, self.image_callback)
        self.lap_pub = rospy.Subscriber("/lapCount", Int32, self.lap_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.heat_pub = rospy.Publisher("/heat", Float32, queue_size=1)
        self.ovt_sub = rospy.Subscriber("/overtake", String, self.ovt_callback)
        self.cant_see_count = 0
        self.twist=Twist()
        self.speed = 0.02
        self.heat = 0
        self.overheated = False
        self.laps = 0
        self.finished_race = False
        self.time = rospy.get_time()
        self.lower_colors = {
            "masking":[
                -1/2,-1*2.55,98*2.55
                ], 
            "blue":[
                58/2,85*2.55,95*2.55
                ]
            }
        self.upper_colors = {
            "masking":[
                1/2,1*2.55,100*2.55
                ], 
            "blue":[
                60/2,95*2.55,100*2.55
                ]}
        self.color_dict = {"not passing":"blue","passing":"masking"}
        self.current_color = "blue"
        self.image = None

    def speed_callback(self, msg):
        # speed here
        # msg type is Int32
        self.speed = msg.data*0.02
        print("current speed:", self.speed)

    def image_callback(self, msg):
        self.image = msg

    def ovt_callback(self, msg):
        print(msg)
        self.current_color = self.color_dict[msg.data]

    def lap_callback(self, msg):
        self.laps = msg.data
        if not self.finished_race:
            self.total_time = rospy.get_time() - self.time
        if self.laps >= 3:
            self.finished_race = True

    def publish_vel(self):
        if not self.finished_race:
            self.cmd_vel_pub.publish(self.twist)
        else:
            print("Race finished! Time: ", self.total_time)
    
    def publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    def image_process(self):      

        print(self.current_color)

        image = self.bridge.compressed_imgmsg_to_cv2(self.image, desired_encoding = "bgr8")

        scale_percent = 20 # percent of original size
        width = int(image.shape[1] * scale_percent / 100)
        height = int(image.shape[0] * scale_percent / 100)
        dim = (width, height)
        
        # image = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)

        # You should be familiar with this
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_bound = numpy.array(self.lower_colors[self.current_color])
        upper_bound = numpy.array(self.upper_colors[self.current_color]) # masking tape, under the stairs (redder light)

        mask = cv2.inRange(hsv,  lower_bound, upper_bound)
        masked = cv2.bitwise_and(image, image, mask=mask)
        cv2.imshow("robovision", masked)

        # clear all but a 20 pixel band near the top of the image
        h, w, d = image.shape
        search_top = int(3 * h /4)
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        

    # Compute the "centroid" and display a red circle to denote it
        M = cv2.moments(mask)
        # print("M00 %d %d" % (M['m00']))

        # print(self.current_color)
        
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])  #50 IS THE DISPLACEMENT VALUE OF THE CENTROID; ADJUST THIS FROM -100 (left of the line) to 0 (directly on top of the line ) to 100 (right of the line)
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 10, (0,0,255), -1)

            # Move at 0.2 M/sec
            # add a turn if the centroid is not in the center
            # Hope for the best. Lots of failure modes.
            err = cx - w/2
            self.twist.linear.x = self.speed
            if self.speed > 0.01:
                self.twist.angular.z = -float(err) / 100 * 0.2
            else:
                self.twist.angular.z = 0
        # else: #SKIDDING
        #     self.cant_see_count += 1
        #     print("YOU LOST CONTROL") #make something that sets angular to zero and linear to a value, and have the car just continue to drive forward (to simulate loss of control)
        #     if self.cant_see_count < 500:
        #         self.twist.linear.x = .012
        #     else:        
        #         self.twist.linear.x = 0
        #         self.twist.angular.z = 0
        #         quit()
        cv2.imshow("image", image)
        cv2.waitKey(1)

    def fuel_tick(self):
        #print("HEAT: ", round(self.heat,2))
        
        if self.heat < 0:
            self.overheated = False
            self.heat = 0
        if self.overheated:
            self.heat += -0.05
            self.heat_pub.publish(self.heat)
            return False
        if self.heat >= 200:
            self.heat = 200
            self.overheated = True
            self.heat_pub.publish(self.heat)
            return False
        # 6,7,8,9 2,4,8,16 /s max 50
        if self.speed == 0.02:
            self.heat += -.15
        if self.speed == 0.04:
            self.heat += -.08
        if self.speed == 0.06:
            self.heat += -.06
        if self.speed == 0.08:
            self.heat += -.04
        if self.speed == 0.12:
            self.heat += .01
        if self.speed == 0.14:
            self.heat += .02
        if self.speed == 0.16:
            self.heat += .04
        if self.speed == 0.18:
            self.heat += .06
        if self.speed == 0.2:
            self.heat += .08

        if self.heat < 0:
            self.heat = 0
      
        self.heat_pub.publish(self.heat)
        return True

if __name__ == '__main__':
    rospy.init_node('follower')
    turtlename = ""
    follower = Follower(turtlename)
    rospy.sleep(3) # to allow the camera to initialize
    while not rospy.is_shutdown():
        follower.image_process()  
        #follower.tick()
        if follower.fuel_tick():
            follower.publish_vel()
        else:
            follower.publish_stop()
        rospy.Rate(10)