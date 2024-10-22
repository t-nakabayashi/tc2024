#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import roslaunch
import rospkg
import tkinter as tk
from tkinter import messagebox, filedialog
from std_msgs.msg import Int32

class MappingProgram:
    def __init__(self):
        # GUIセットアップ
        self.root = tk.Tk()
        self.root.title("Mapping Program")

        # マップ作成ボタン
        self.create_map_button = tk.Button(self.root, text="マップ作成", command=self.create_map)
        self.create_map_button.pack(pady=10)

        # マップ後処理ボタン
        self.post_process_button = tk.Button(self.root, text="マップ後処理", command=self.post_process)
        self.post_process_button.pack(pady=10)

        # ROS初期化
        rospy.init_node('mapping_program', anonymous=True)

        self.base_dir = ""
        self.current_map_index = 0
        self.launch = None

        self.root.mainloop()

    def create_map(self):
        # 親ディレクトリを選択
        parent_dir = filedialog.askdirectory(title="親ディレクトリを選択してください")

        if not parent_dir:
            return

        if os.path.exists(parent_dir):
            # 警告メッセージを表示
            if not messagebox.askokcancel("警告", "ディレクトリが既に存在します。全てのファイルを削除してもよろしいですか？"):
                return
            # 既存のファイルを削除
            for root, dirs, files in os.walk(parent_dir):
                for file in files:
                    os.remove(os.path.join(root, file))
        else:
            # 親ディレクトリが存在しない場合は作成
            os.makedirs(parent_dir)
            rospy.loginfo('親ディレクトリを作成しました: {}'.format(parent_dir))

        # ベースディレクトリを設定
        self.base_dir = parent_dir

        # maps.txt作成
        maps_txt_path = os.path.join(self.base_dir, "maps.txt")
        
        # maps.txtを作成する前に、ディレクトリが存在するかを確認し、必要に応じて作成
        if not os.path.exists(self.base_dir):
            os.makedirs(self.base_dir)

        with open(maps_txt_path, 'w') as maps_file:
            rospy.loginfo("maps.txtファイルを作成しました。")

        self.create_map_directory(self.current_map_index)

    def create_map_directory(self, index):
        # マップディレクトリ作成
        map_dir = os.path.join(self.base_dir, 'map{}'.format(index))
        if not os.path.exists(map_dir):
            os.makedirs(map_dir)
            rospy.loginfo('ディレクトリを作成しました: {}'.format(map_dir))
        else:
            rospy.loginfo('ディレクトリは既に存在します: {}'.format(map_dir))

        # maps.txtにディレクトリパスを追記
        maps_txt_path = os.path.join(self.base_dir, "maps.txt")
        with open(maps_txt_path, 'a') as maps_file:
            maps_file.write(os.path.abspath(map_dir) + '\n')

        # シンボリックリンクの作成
        symlink_path = '/home/nakaba/catkin_ws/src/FAST_LIO/PCD'
        if os.path.islink(symlink_path) or os.path.exists(symlink_path):
            rospy.loginfo('既存のシンボリックリンクまたはファイルを削除します: {}'.format(symlink_path))
            os.remove(symlink_path)  # シンボリックリンクが存在すれば削除

        os.symlink(os.path.abspath(map_dir), symlink_path)
        rospy.loginfo('シンボリックリンクを作成しました: {} -> {}'.format(symlink_path, map_dir))

        # launchファイルを起動
        self.start_launch_file()

    def start_launch_file(self):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # パッケージのパスを取得して、ランチファイルのフルパスを作成
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('tc2024')
        launch_file = os.path.join(package_path, 'launch/2-1_mapping_fastlio.launch')

        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        self.launch.start()
        rospy.loginfo('ランチファイルを起動しました。')

    def post_process(self):
        # maps.txtを選択
        maps_txt_path = filedialog.askopenfilename(title="maps.txtを選択してください", filetypes=[("Text files", "*.txt")])

        if not maps_txt_path or not maps_txt_path.endswith('maps.txt'):
            messagebox.showerror("エラー", "正しいmaps.txtファイルを選択してください。")
            return

        with open(maps_txt_path, 'r') as maps_file:
            map_dirs = maps_file.readlines()

        for map_dir in map_dirs:
            map_dir = map_dir.strip()
            if os.path.exists(map_dir):
                # ROSパラメータを設定
                rospy.set_param('map_path', map_dir)

                # 2-2_pcd_cleaning.launchの実行
                self.run_launch_file('2-2_pcd_cleaning.launch')

                # 2-3_pcd_to_2dmap.launchの実行
                self.run_launch_file('2-3_pcd_to_2dmap.launch')
            else:
                rospy.logerr("ディレクトリが存在しません: {}".format(map_dir))

    def run_launch_file(self, launch_file_name):
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        rospack = rospkg.RosPack()
        package_path = rospack.get_path('tc2024')
        launch_file = os.path.join(package_path, 'launch', launch_file_name)

        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        launch.start()
        rospy.loginfo('{} を起動しました。'.format(launch_file_name))
        launch.spin()

if __name__ == '__main__':
    MappingProgram()
