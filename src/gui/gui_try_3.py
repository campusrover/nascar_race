import tkinter as tk
from tkinter import ttk
import rospy
import random
from std_msgs.msg import Int32,Float32

heat = 1337

def update():
    l.config(text=heat)
    l.update()

    # run this again after 1000 ms
    root.after(1000, update)

def heat_cb(msg):
    global heat
    print(msg)
    heat = msg.data
    l.config(text=heat)
    l.update()

rospy.init_node("tkinter")

heat_sub = rospy.Subscriber('robc/heat', Float32, heat_cb)

root = tk.Tk()
l = tk.Label(text='0')
l.pack()
root.after(1000, update)
root.mainloop()


# https://stackoverflow.com/questions/27123676/how-to-update-python-tkinter-window