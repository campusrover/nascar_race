import tkinter as tk
from tkinter import ttk
import rospy
from std_msgs.msg import Int32,Float32,String

rospy.init_node("tkinter")

controller = tk.Tk()

controller.title("NATBAR Qualifier Controller")

speed = 0
lane = "blue"
#NEED TO DO: PRINT SPEED TO GUI
speed_output = 1337.420
speed_label = tk.Label(controller, text = speed_output, font = "Arial")

def speed_callback(msg):
    global speed_output
    global speed_label
    global controller
    speed_output = msg.data
    speed_label = tk.Label(controller, text = speed_output, font = "Arial")

#backend stuff
speed_pub = rospy.Publisher('robc/speed', Int32, queue_size = 5)
lane_pub = rospy.Publisher('robc/overtake', String, queue_size = 5)
heat_sub = rospy.Subscriber('robc/heat', Float32, speed_callback)

#START - controller
####CANVAS LAYER
canvas = tk.Canvas(controller, width=300, height=100)
canvas.grid(columnspan=3)

#Logo + Header
logo = tk.PhotoImage(file = "/my_ros_data/catkin_ws/src/nascar/src/gui/logo.png")
logo = logo.zoom(10)
logo = logo.subsample(70)
logo_label= tk.Label(image = logo)
logo_label.image = logo
logo_label.grid(column=1,row=0)

controller_title = tk.Label(controller, text = "\nNATBAR 2022\nCONTROLLER", font = ("Arial",18,"bold"))
controller_title.grid(column=1,row=1)
controller_rules = tk.Label(controller, text = "Press ▲ to Increase Speed\nPress ▼ to Decrease Speed\nPress ◀ ▶ to Switch Lanes\n", font = "Arial")
controller_rules.grid(column=1,row=2)

#### BUTTON LAYER
##SPEED UP 
speed_up = tk.StringVar()
speed_up_btn = tk.Button(controller, textvariable=speed_up, command=lambda:faster(), font = "Arial", bg = "black", fg = "white", height=1, width=5)
##btn fspeed_upn START
def faster():
    global speed
    if speed<10:
        speed = speed+1
        speed_pub.publish(speed)
    else:
        speed = 10
        speed_pub.publish(speed)
##btn fspeed_upn END
speed_up.set("▲")
speed_up_btn.grid(column=1,row=3)

## SLOW DOWN
slow_down = tk.StringVar()
slow_down_btn = tk.Button(controller, textvariable=slow_down, command=lambda:slower(), font = "Arial", bg = "black", fg = "white", height=1, width=5)
##btn fspeed_upn START
def slower():
    global speed
    if speed>0:
        speed = speed-1
        speed_pub.publish(speed)
    else:
        speed = 0
        speed_pub.publish(speed)
##btn fspeed_upn END
slow_down.set("▼")
slow_down_btn.grid(column=1,row=4)

lanechanger = tk.StringVar()
change_lanes_btn = tk.Button(controller, textvariable=lanechanger, command = lambda:change_lanes(), font = "Arial", bg = "black", fg = "white", height = 1, width = 5)
def change_lanes():
    global lane
    if lane=="blue":
        lane="masking"
        lane_pub.publish("passing")
    elif lane=="masking":
        lane="blue"
        lane_pub.publish("not passing")
lanechanger.set("◀ ▶")
change_lanes_btn.grid(column=1,row=5)

####CANVAS LAYER
canvas = tk.Canvas(controller,width=300,height=50)
canvas.grid(columnspan=3)

controller.mainloop()
#END - controller