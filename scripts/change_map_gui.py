#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32
try:
    import tkinter as tk
except ImportError:
    import Tkinter as tk

class ChangeMapGUI:
    def __init__(self):
        rospy.init_node('change_map_gui')

        self.pub = rospy.Publisher('change_map', Int32, queue_size=10)

        self.root = tk.Tk()
        self.root.title('Change Map GUI')

        self.var = tk.IntVar(value=0)

        self.label = tk.Label(self.root, text='Change Map Value:')
        self.label.pack()

        self.radio0 = tk.Radiobutton(self.root, text='0', variable=self.var, value=0, command=self.publish_value)
        self.radio0.pack()

        self.radio1 = tk.Radiobutton(self.root, text='1', variable=self.var, value=1, command=self.publish_value)
        self.radio1.pack()

        self.button_quit = tk.Button(self.root, text='Quit', command=self.quit)
        self.button_quit.pack()

        self.root.protocol("WM_DELETE_WINDOW", self.quit)

        self.publish_value()

    def publish_value(self):
        value = self.var.get()
        rospy.loginfo('change_mapに値を送信: {}'.format(value))
        msg = Int32(data=value)
        self.pub.publish(msg)

    def quit(self):
        self.root.quit()

    def spin(self):
        while not rospy.is_shutdown():
            self.root.update()
            rospy.sleep(0.1)

if __name__ == '__main__':
    gui = ChangeMapGUI()
    gui.spin()
