#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import tkinter as tk
import subprocess
import time
import signal
import threading
import atexit
import rospy
from std_msgs.msg import Int32

# 各launchファイルの情報
LOCARIZATION_LAUNCH = ["roslaunch", "tc2024", "3-1_locarization_fastlio_1st.launch"]
NAVIGATION_LAUNCH_TEMPLATE = ["roslaunch", "tc2024", "4_run_1st.launch", "start_num:="]
LOCARIZATION_NEXT_LAUNCH = ["roslaunch", "tc2024", "3-1_locarization_fastlio_next.launch"]
NAVIGATION_NEXT_LAUNCH = ["roslaunch", "tc2024", "4_run_next.launch", "start_num:="]

processes = {}

def start_launch(name, command):
    if name in processes and processes[name].poll() is None:
        print(f"{name} is already running.")
        return
    print(f"Starting {name}...")
    processes[name] = subprocess.Popen(command)

def stop_launch(name):
    if name in processes and processes[name].poll() is None:
        print(f"Stopping {name}...")
        processes[name].send_signal(signal.SIGINT)
        processes[name].wait()
        print(f"{name} stopped.")
    else:
        print(f"{name} is not running or already stopped.")

def stop_all_launches():
    print("Stopping all launches...")
    for name, process in processes.items():
        if process.poll() is None:
            print(f"Force stopping {name}...")
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print(f"{name} did not stop in time. Killing it.")
                process.kill()
    print("All launches stopped.")

atexit.register(stop_all_launches)

def start_locarization():
    start_launch("locarization", LOCARIZATION_LAUNCH)

def restart_locarization():
    stop_launch("locarization")
    start_launch("locarization", LOCARIZATION_NEXT_LAUNCH)

def stop_locarization():
    stop_launch("locarization")

def start_navigation_with_waypoint():
    waypoint = waypoint_entry.get() or "0"
    if not waypoint.isdigit():
        print("Invalid waypoint number. Please enter a numeric value.")
        return
    navigation_command = NAVIGATION_LAUNCH_TEMPLATE + [f"start_num:={waypoint}"]
    start_launch("navigation", navigation_command)

def restart_navigation():
    stop_launch("navigation")
    waypoint = "1"
    navigation_command = NAVIGATION_NEXT_LAUNCH + ["start_num:=1"]
    start_launch("navigation", navigation_command)

def restart_all():
    stop_navigation()
    restart_locarization()
    restart_navigation()

def stop_navigation():
    stop_launch("navigation")

rospy.init_node('gui_node', anonymous=True, disable_signals=True)
manual_start_pub = rospy.Publisher('/manual_start', Int32, queue_size=1)

def send_manual_start():
    manual_start_pub.publish(1)
    print("Manual start signal sent.")

# グローバル変数で最新のgpt_output値とwaypoint_time値を保持
gpt_output_value = 0
waypoint_time_value = 0

def gpt_output_callback(data):
    """/gpt_outputトピックから値を受け取るコールバック関数"""
    global gpt_output_value
    if data.data == 99:
        gpt_output_value = "未定義"
    elif data.data == 2:
        gpt_output_value = "静止障害物（カラーコーン）"
    elif data.data == 1:
        gpt_output_value = "人、ロボット"
    elif data.data == 0:
        gpt_output_value = "誤検知"



def waypoint_time_callback(data):
    """/waypoint_timeトピックから値を受け取るコールバック関数"""
    global waypoint_time_value
    waypoint_time_value = data.data

# /gpt_outputと/waypoint_timeのサブスクライバーを作成
rospy.Subscriber('/gpt_output', Int32, gpt_output_callback)
rospy.Subscriber('/waypoint_time', Int32, waypoint_time_callback)

def update_labels():
    """GUIのラベルを最新のgpt_output値とwaypoint_time値に更新"""
    gpt_output_label.config(text=f"/gpt_output: {gpt_output_value}")
    waypoint_time_label.config(text=f"/waypoint_time: {waypoint_time_value}")
    root.after(100, update_labels)  # 100msごとに更新

root = tk.Tk()
root.title("ROS Launch Manager")
root.geometry("1000x400")
root.attributes("-topmost", True)

font_size = 22  # Base font size

locarization_frame = tk.Frame(root)
locarization_frame.pack(pady=10, anchor="w")
start_locarization_button = tk.Button(locarization_frame, text="Start Locarization", command=start_locarization, width=20, font=("Arial", font_size))
start_locarization_button.pack(side="left", padx=5)
stop_locarization_button = tk.Button(locarization_frame, text="Stop Locarization", command=stop_locarization, width=20, font=("Arial", font_size))
stop_locarization_button.pack(side="left", padx=5)
restart_locarization_button = tk.Button(locarization_frame, text="Restart Locarization", command=restart_locarization, width=20, font=("Arial", font_size))
restart_locarization_button.pack(side="left", padx=5)

waypoint_frame = tk.Frame(root)
waypoint_frame.pack(pady=10, anchor="w")
waypoint_label = tk.Label(waypoint_frame, text="Waypoint (default: 0):", width=20, anchor="w", font=("Arial", font_size))
waypoint_label.pack(side="left")
waypoint_entry = tk.Entry(waypoint_frame, font=("Arial", font_size))
waypoint_entry.insert(0, "0")
waypoint_entry.pack(side="left", padx=5)

navigation_frame = tk.Frame(root)
navigation_frame.pack(pady=10, anchor="w")
start_navigation_button = tk.Button(navigation_frame, text="Start Navigation", command=start_navigation_with_waypoint, width=20, font=("Arial", font_size))
start_navigation_button.pack(side="left", padx=5)
stop_navigation_button = tk.Button(navigation_frame, text="Stop Navigation", command=stop_navigation, width=20, font=("Arial", font_size))
stop_navigation_button.pack(side="left", padx=5)
restart_navigation_button = tk.Button(navigation_frame, text="Restart Navigation", command=restart_navigation, width=20, font=("Arial", font_size))
restart_navigation_button.pack(side="left", padx=5)

manual_start_frame = tk.Frame(root)
manual_start_frame.pack(pady=10, anchor="w")
manual_start_button = tk.Button(manual_start_frame, text="Manual Start", command=send_manual_start, width=20, bg='green', fg='white', font=("Arial", font_size))
manual_start_button.pack(side="left", padx=5)
restart_navigation_button = tk.Button(manual_start_frame, text="Restart all", command=restart_all, width=20, font=("Arial", font_size))
restart_navigation_button.pack(side="left", padx=5)

# gpt_outputとwaypoint_timeの表示
output_frame = tk.Frame(root)
output_frame.pack(pady=20, anchor="w")
gpt_output_label = tk.Label(output_frame, text="/gpt_output: 0", font=("Arial", font_size), wraplength=600, justify="left")
gpt_output_label.pack()
waypoint_time_label = tk.Label(output_frame, text="/waypoint_time: 0", font=("Arial", font_size), wraplength=600, justify="left")
waypoint_time_label.pack()

root.protocol("WM_DELETE_WINDOW", lambda: (stop_all_launches(), root.destroy()))

update_labels()  # 定期的にラベルを更新

root.mainloop()
