#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import tf
import csv
import os
from datetime import datetime
from tkinter import Tk, Label, Entry, Button, StringVar, filedialog
from geometry_msgs.msg import PoseStamped


class WaypointEditor:
    def __init__(self, master):
        self.master = master
        self.master.title("Waypoint Editor")
        self.master.attributes('-topmost', True)

        # CSVファイル関連
        self.csv_file = None
        self.waypoints = []
        self.selected_num = None

        # RVizからのGoal情報
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        # GUI要素
        Label(master, text="Waypoint CSV:").grid(row=0, column=0)
        self.csv_label = StringVar()
        self.csv_label.set("No file selected")
        Label(master, textvariable=self.csv_label).grid(row=0, column=1)
        Button(master, text="Open", command=self.load_csv).grid(row=0, column=2)

        Label(master, text="Select NUM:").grid(row=1, column=0)
        self.num_entry = Entry(master)
        self.num_entry.grid(row=1, column=1)
        Button(master, text="Set NUM", command=self.set_num).grid(row=1, column=2)

        self.status_label = StringVar()
        self.status_label.set("Status: Waiting for input...")
        Label(master, textvariable=self.status_label).grid(row=2, column=0, columnspan=3)

        Button(master, text="Save CSV", command=self.save_csv).grid(row=3, column=0, columnspan=3)

    def load_csv(self):
        initial_dir = "/home/nakaba/catkin_ws/src/FAST_LIO/PCD"
        file_path = filedialog.askopenfilename(initialdir=initial_dir, filetypes=[("CSV Files", "*.csv")])
        if not file_path:
            return
        self.csv_file = file_path
        self.waypoints = []
        with open(self.csv_file, mode='r') as file:
            reader = csv.DictReader(file)
            for row in reader:
                rospy.loginfo(f"Loaded row: {row}")  # デバッグ: 読み込んだ行を出力
                self.waypoints.append(row)
        self.csv_label.set(f"Loaded: {file_path}")
        self.status_label.set("Status: CSV loaded. Select NUM.")

    def save_csv(self):
        if not self.csv_file or not self.waypoints:
            self.status_label.set("Error: No CSV loaded or no waypoints to save.")
            return
        
        # 保存前にバックアップ
        self.backup_csv()

        with open(self.csv_file, mode='w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=self.waypoints[0].keys())
            writer.writeheader()
            writer.writerows(self.waypoints)
        self.status_label.set("Status: CSV saved successfully.")

    def backup_csv(self):
        if not self.csv_file:
            return

        # 元ファイルのディレクトリを取得
        dir_name = os.path.dirname(self.csv_file)
        backup_dir = os.path.join(dir_name, "old_waypoint")

        # ディレクトリが存在しない場合は作成
        if not os.path.exists(backup_dir):
            os.makedirs(backup_dir)

        # 現在の時刻をファイル名に追加
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_file = os.path.join(backup_dir, f"waypoint_backup_{timestamp}.csv")

        # ファイルをコピーしてバックアップ
        with open(self.csv_file, mode='r') as original, open(backup_file, mode='w', newline='') as backup:
            backup.write(original.read())
        
        rospy.loginfo(f"Backup saved to: {backup_file}")
        self.status_label.set(f"Backup saved: {backup_file}")

    def set_num(self):
        try:
            self.selected_num = int(self.num_entry.get())
            self.status_label.set(f"Status: NUM {self.selected_num} selected. Set a goal in RViz.")
        except ValueError:
            self.status_label.set("Error: Invalid NUM entered.")

    def goal_callback(self, msg):
        if self.selected_num is None:
            rospy.logwarn("No NUM selected. Use the GUI to select a waypoint.")
            self.status_label.set("Error: No NUM selected.")
            return

        # Extract position and orientation
        position = msg.pose.position
        orientation = msg.pose.orientation

        # Update the selected waypoint
        updated = False
        for waypoint in self.waypoints:
            try:
                if int(waypoint.get('num', 0)) == self.selected_num:
                    waypoint['x'] = position.x
                    waypoint['y'] = position.y
                    waypoint['z'] = position.z
                    waypoint['q3'] = orientation.z
                    waypoint['q4'] = orientation.w
                    updated = True
                    rospy.loginfo(f"Waypoint {self.selected_num} updated: {waypoint}")
                    break
            except KeyError as e:
                rospy.logerr(f"KeyError: {e}. Check CSV file format.")
                self.status_label.set(f"Error: Missing key in CSV - {e}")
                return

        if updated:
            self.status_label.set(f"Status: Waypoint {self.selected_num} updated.")
        else:
            self.status_label.set(f"Error: NUM {self.selected_num} not found.")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    rospy.init_node('waypoint_editor_gui')

    # Initialize the GUI
    root = Tk()
    editor = WaypointEditor(root)

    rospy.loginfo("Starting Waypoint Editor GUI...")
    root.mainloop()
