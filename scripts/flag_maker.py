#!/usr/bin/env python3
# coding=utf-8

import rospy
import tkinter as tk
from tkinter import ttk
from std_msgs.msg import Int32
import signal
import sys

class ROSParamGUI:
    def __init__(self, master):
        self.master = master
        master.title("ROS Parameter Controller")

        # ウィンドウを常に最前面に表示
        master.attributes("-topmost", True)

        # パラメータとそのラベル
        self.params = {
            '/right_is_open': {'label': 'Right Is Open', 'default': 0},
            '/left_is_open': {'label': 'Left Is Open', 'default': 0},
            '/line_is_stop': {'label': 'Line Is Stop', 'default': 0},
            '/signal_is_stop': {'label': 'Signal Is Stop', 'default': 0},
            '/isnot_skipnum': {'label': 'Is Not Skip Num', 'default': 0},
            '/change_map': {'label': 'Change Map', 'default': 0}
        }

        self.entries = {}
        self.publishers = {}
        self.counts = {}

        row = 0
        for param, info in self.params.items():
            label = ttk.Label(master, text=info['label'])
            label.grid(row=row, column=0, padx=10, pady=5, sticky=tk.W)

            var = tk.IntVar()
            var.set(info['default'])
            self.entries[param] = var

            checkbox = ttk.Checkbutton(master, variable=var)
            checkbox.grid(row=row, column=1, padx=10, pady=5)
            row += 1

            # 各パラメータ用のトピックを作成
            topic_name = param.replace('/', '')  # '/'を取り除いたトピック名
            self.publishers[param] = rospy.Publisher(topic_name, Int32, queue_size=10)

            # 各パラメータに対して1が送信された回数をカウント
            self.counts[param] = 0

        # 0.5秒周期でパブリッシュを行うタイマー
        rospy.Timer(rospy.Duration(0.5), self.publish_params)

    def update_parameters(self):
        try:
            for param, var in self.entries.items():
                value = var.get()
                # 値が更新された際にトピックに値をpublish
                self.publishers[param].publish(value)
            tk.messagebox.showinfo("Success", "Parameters updated and published successfully!")
        except Exception as e:
            tk.messagebox.showerror("Error", f"Failed to update parameters:\n{e}")

    def publish_params(self, event):
        # 0.5秒周期で全てのパラメータをパブリッシュ
        for param, var in self.entries.items():
            value = var.get()
            self.publishers[param].publish(value)

            # 1が送信された場合にカウントを増やし、3回送信したら0に戻す
            if value == 1:
                self.counts[param] += 1
                rospy.loginfo(f"{param} has been set to 1. Count: {self.counts[param]}")

                if self.counts[param] >= 3:
                    rospy.loginfo(f"{param} has been set to 1 three times. Resetting to 0.")
                    var.set(0)
                    self.publishers[param].publish(0)
                    self.counts[param] = 0  # カウントリセット

            # 値が0の場合、カウントをリセット
            elif value == 0:
                self.counts[param] = 0

# Ctrl+Cで正常終了できるように設定
def signal_handler(sig, frame):
    rospy.loginfo("Shutting down ROSParamGUI")
    sys.exit(0)

def main():
    rospy.init_node('ros_param_gui', anonymous=True)

    # Ctrl+Cでプログラム終了をキャッチ
    signal.signal(signal.SIGINT, signal_handler)

    root = tk.Tk()
    gui = ROSParamGUI(root)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        rospy.loginfo("Exiting program")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
