import tkinter as tk
from tkinter import ttk
import rospy
from std_msgs.msg import Int32,Float32

speed_label = None
speed = None

def speed_callback(msg):
    print("got calledback", msg.data)
    global speed
    speed = msg.data

def label_loop():
        global speed_label

        # im=Image.fromstring('L', (data.shape[1],data.shape[0]), data.astype('b').tostring())
        # photo = ImageTk.PhotoImage(image=im)
        # canvas.create_image(0,0,image=photo,anchor=Tkinter.NW)

        speed_label = tk.Label(controller, text = speed, font = "Arial")

        # try 2 refresh?
        controller['speed_printer'] = speed_label

        controller.update()

        # times+=1
        # if times%33==0:
        #         print "%.02f FPS"%(times/(time.clock()-timestart))

        # after 10ms, run img loop
        controller.after(10,label_loop)



#backend stuff
speed_pub = rospy.Publisher('robc/speed', Int32, queue_size = 5)
heat_sub = rospy.Subscriber('robc/heat', Float32, speed_callback)

#START - controller
controller = tk.Tk()
controller.title("NATBAR Qualifier Controller")

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

controller_title = tk.Label(controller, text = "NATBAR 2022 \n CONTROLLER", font = ("Arial",18,"bold"))
controller_title.grid(column=1,row=1)
controller_rules = tk.Label(controller, text = "Press ▲ to Increase Speed\nPress ▼ to Decrease Speed", font = "Arial")
controller_rules.grid(columnspan=3,column=0,row=2)

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

####CANVAS LAYER
canvas = tk.Canvas(controller,width=300,height=50)
canvas.grid(columnspan=3)

speed_printer = speed_label
# speed_printer.grid(columnspan=3,column=0,row=5)


controller.after(0,label_loop)
data = 420.69

while True:
        #print "do work again and again, change data"
        controller.update()
        controller.update_idletasks()


# https://stackoverflow.com/questions/24849265/how-do-i-create-an-automatically-updating-gui-using-tkinter