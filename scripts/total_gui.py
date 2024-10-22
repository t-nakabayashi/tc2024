#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import rospkg
from std_msgs.msg import Int32
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import threading
import shutil
import subprocess
import signal
import sys

class MappingProgram:
    def __init__(self):
        rospy.init_node('mapping_program', anonymous=True)

        self.change_map_value = 0
        self.prev_change_map_value = 0

        self.base_dir = None
        self.maps_txt_path = None
        self.current_map_index = 0

        rospy.Subscriber('change_map', Int32, self.change_map_callback)

    def set_base_directory(self, base_dir):
        self.base_dir = base_dir
        if not os.path.exists(self.base_dir):
            os.makedirs(self.base_dir)
            rospy.loginfo(f'ベースディレクトリを作成しました: {self.base_dir}')
        else:
            rospy.loginfo(f'ベースディレクトリは既に存在します: {self.base_dir}')

        # maps.txtのパス
        self.maps_txt_path = os.path.join(self.base_dir, 'maps.txt')
        # maps.txtを初期化
        with open(self.maps_txt_path, 'w') as f:
            f.write('')  # 空ファイルとして作成
        rospy.loginfo(f'maps.txtを初期化しました: {self.maps_txt_path}')

        self.current_map_index = 0
        self.create_map_directory(self.current_map_index)

    def create_map_directory(self, index):
        if self.base_dir is None:
            rospy.logerr("ベースディレクトリが設定されていません。")
            messagebox.showerror("エラー", "ベースディレクトリが設定されていません。マップ作成を先に行ってください。")
            return

        # ベースディレクトリの中にmap0, map1...を作成
        map_dir = os.path.join(self.base_dir, f'map{index}')
        if not os.path.exists(map_dir):
            os.makedirs(map_dir)
            rospy.loginfo(f'ディレクトリを作成しました: {map_dir}')
        else:
            rospy.loginfo(f'Directory already exists: {map_dir}')

        # maps.txtに絶対パスを記録
        abs_map_dir = os.path.abspath(map_dir)
        with open(self.maps_txt_path, 'a') as f:
            f.write(abs_map_dir + '\n')
        rospy.loginfo(f'maps.txtにパスを記録しました: {abs_map_dir}')

        # シンボリックリンクの作成（必要に応じて修正）
        symlink_path = '/home/nakaba/catkin_ws/src/FAST_LIO/PCD'  # 必要に応じて修正
        if os.path.islink(symlink_path) or os.path.exists(symlink_path):
            rospy.loginfo(f'既存のシンボリックリンクまたはファイルを削除します: {symlink_path}')
            os.remove(symlink_path)  # シンボリックリンクまたはファイルが存在すれば削除

        try:
            os.symlink(abs_map_dir, symlink_path)
            rospy.loginfo(f'シンボリックリンクを作成しました: {symlink_path} -> {map_dir}')
        except Exception as e:
            rospy.logerr(f'シンボリックリンクの作成に失敗しました: {e}')
            messagebox.showerror("エラー", f"シンボリックリンクの作成に失敗しました。\n詳細: {e}")
            return

        # 2-1_mapping_fastlio.launchの実行
        mapping_launch_file = os.path.join(rospkg.RosPack().get_path('tc2024'), 'launch/2-1_mapping_fastlio.launch')
        if not os.path.isfile(mapping_launch_file):
            rospy.logerr(f'ランチファイルが存在しません: {mapping_launch_file}')
            messagebox.showerror("エラー", f"ランチファイルが存在しません: {mapping_launch_file}")
            return

        mapping_cmd = [
            'roslaunch',
            mapping_launch_file,
            f'map_path:={map_dir}'
        ]

        rospy.loginfo(f'ランチファイルを起動します: {" ".join(mapping_cmd)}')
        process = subprocess.Popen(mapping_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate()

        if process.returncode != 0:
            rospy.logerr(f'ランチファイル実行中にエラーが発生しました: {mapping_launch_file}\n{stderr.decode()}')
            messagebox.showerror("エラー", f"ランチファイル実行中にエラーが発生しました: {mapping_launch_file}\n詳細: {stderr.decode()}")
            return
        else:
            rospy.loginfo(f'ランチファイルを正常に終了しました: {mapping_launch_file}')

    def change_map_callback(self, msg):
        self.change_map_value = msg.data
        rospy.loginfo(f'change_mapの値を受信: {self.change_map_value}')
        # ここではマップディレクトリの作成を行わず、単にログを記録するのみとします
        # 必要に応じて他の処理を追加してください

class ROSHandler:
    def __init__(self, mapping_program):
        self.mapping_program = mapping_program
        self.package_path = rospkg.RosPack().get_path('tc2024')

    def execute_post_processing(self, maps_txt_path):
        if not os.path.isfile(maps_txt_path):
            rospy.logerr(f'指定されたファイルが存在しません: {maps_txt_path}')
            messagebox.showerror("エラー", "指定されたファイルが存在しません。")
            return

        # maps.txtを読み込む
        with open(maps_txt_path, 'r') as f:
            map_dirs = [line.strip() for line in f.readlines() if line.strip()]

        if not map_dirs:
            rospy.logerr('maps.txtに有効なディレクトリパスが含まれていません。')
            messagebox.showerror("エラー", "maps.txtに有効なディレクトリパスが含まれていません。")
            return

        for map_dir in map_dirs:
            rospy.loginfo(f'処理対象のマップディレクトリ: {map_dir}')

            # Launchファイルのフルパス
            cleaning_launch_file = os.path.join(self.package_path, 'launch/2-2_pcd_cleaning.launch')
            if not os.path.isfile(cleaning_launch_file):
                rospy.logerr(f'ランチファイルが存在しません: {cleaning_launch_file}')
                messagebox.showerror("エラー", f"ランチファイルが存在しません: {cleaning_launch_file}")
                continue

            # Launchファイルをフルパスで実行
            cleaning_cmd = [
                'roslaunch',
                cleaning_launch_file,
                f'map_path:={map_dir}'
            ]

            rospy.loginfo(f'ランチファイルを起動します: {" ".join(cleaning_cmd)}')
            process = subprocess.Popen(cleaning_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = process.communicate()

            if process.returncode != 0:
                rospy.logerr(f'ランチファイル実行中にエラーが発生しました: {cleaning_launch_file}\n{stderr.decode()}')
                messagebox.showerror("エラー", f"ランチファイル実行中にエラーが発生しました: {cleaning_launch_file}\n詳細: {stderr.decode()}")
                continue
            else:
                rospy.loginfo(f'ランチファイルを正常に終了しました: {cleaning_launch_file}')

            # 2-3_pcd_to_2dmap.launchの実行
            to_2dmap_launch_file = os.path.join(self.package_path, 'launch/2-3_pcd_to_2dmap.launch')
            if not os.path.isfile(to_2dmap_launch_file):
                rospy.logerr(f'ランチファイルが存在しません: {to_2dmap_launch_file}')
                messagebox.showerror("エラー", f"ランチファイルが存在しません: {to_2dmap_launch_file}")
                continue

            # Launchファイルをフルパスで実行
            to_2dmap_cmd = [
                'roslaunch',
                to_2dmap_launch_file,
                f'map_path:={map_dir}'
            ]

            rospy.loginfo(f'ランチファイルを起動します: {" ".join(to_2dmap_cmd)}')
            process = subprocess.Popen(to_2dmap_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            stdout, stderr = process.communicate()

            if process.returncode != 0:
                rospy.logerr(f'ランチファイル実行中にエラーが発生しました: {to_2dmap_launch_file}\n{stderr.decode()}')
                messagebox.showerror("エラー", f"ランチファイル実行中にエラーが発生しました: {to_2dmap_launch_file}\n詳細: {stderr.decode()}")
                continue
            else:
                rospy.loginfo(f'ランチファイルを正常に終了しました: {to_2dmap_launch_file}')

        messagebox.showinfo("完了", "マップ後処理が完了しました。")

class ROSParamController:
    def __init__(self, master, publish_enabled_var):
        self.publish_enabled_var = publish_enabled_var
        self.params = {
            '/right_is_open': 'right_is_open',
            '/left_is_open': 'left_is_open',
            '/line_is_stop': 'line_is_stop',
            '/signal_is_stop': 'signal_is_stop',
            '/isnot_skipnum': 'isnot_skipnum',
            '/change_map': 'change_map'
        }
        self.publishers = {}
        self.vars = {}
        self.counts = {}

        self.setup_publishers(master)

    def setup_publishers(self, master):
        for param, topic in self.params.items():
            self.publishers[param] = rospy.Publisher(topic, Int32, queue_size=10)
            self.vars[param] = tk.IntVar(value=0)
            self.counts[param] = 0

    def publish_param(self, param, value):
        if self.publish_enabled_var.get():
            self.publishers[param].publish(Int32(data=value))
            rospy.loginfo(f'パラメータ "{param}" に値 {value} をパブリッシュしました。')

    def handle_param_change(self, param, value):
        self.publish_param(param, value)
        if value == 1:
            self.counts[param] += 1
            rospy.loginfo(f"{param} has been set to 1. Count: {self.counts[param]}")

            if self.counts[param] >= 3:
                rospy.loginfo(f"{param} has been set to 1 three times. Resetting to 0.")
                self.vars[param].set(0)
                self.publish_param(param, 0)
                self.counts[param] = 0
        else:
            self.counts[param] = 0

class GUI:
    def __init__(self, root, mapping_program, ros_handler, ros_param_controller):
        self.root = root
        self.mapping_program = mapping_program
        self.ros_handler = ros_handler
        self.ros_param_controller = ros_param_controller

        self.root.title("Mapping and ROS Control Program")

        # Create Map Button
        self.create_map_button = ttk.Button(self.root, text="マップ作成", command=self.create_map)
        self.create_map_button.pack(pady=10)

        # Post Processing Button
        self.post_processing_button = ttk.Button(self.root, text="マップ後処理", command=self.post_processing)
        self.post_processing_button.pack(pady=10)

        # Status Label
        self.status_label = ttk.Label(self.root, text="ステータス: 待機中")
        self.status_label.pack(pady=10)

        # Separator
        separator = ttk.Separator(self.root, orient='horizontal')
        separator.pack(fill='x', pady=10)

        # Publishing Controls Frame
        self.publishing_frame = ttk.LabelFrame(self.root, text="トピックパブリッシュ設定")
        self.publishing_frame.pack(fill='both', expand=True, padx=10, pady=10)

        # Publish Enabled Checkbox
        self.publish_checkbox = ttk.Checkbutton(
            self.publishing_frame,
            text="トピックをパブリッシュする",
            variable=self.ros_param_controller.publish_enabled_var
        )
        self.publish_checkbox.pack(anchor='w', pady=5)

        # Parameters Checkboxes
        self.params_label = ttk.Label(self.publishing_frame, text="ROSパラメータ:")
        self.params_label.pack(anchor='w', pady=(10, 0))

        self.params = ['/right_is_open', '/left_is_open', '/line_is_stop',
                      '/signal_is_stop', '/isnot_skipnum', '/change_map']
        self.param_checkboxes = {}
        for param in self.params:
            checkbox = ttk.Checkbutton(
                self.publishing_frame,
                text=param,
                variable=self.ros_param_controller.vars[param],
                command=lambda p=param: self.param_changed(p)
            )
            checkbox.pack(anchor='w', padx=20)
            self.param_checkboxes[param] = checkbox

    def create_map(self):
        selected_dir = filedialog.askdirectory(title="親ディレクトリを選択または作成")
        if not selected_dir:
            messagebox.showerror("エラー", "親ディレクトリが選択されていません。")
            return

        if os.path.exists(selected_dir):
            proceed = messagebox.askokcancel(
                "警告",
                "指定した親ディレクトリは既に存在します。\n内部の全ファイルとディレクトリが削除されます。\n続行しますか？"
            )
            if proceed:
                # ディレクトリ内の全ファイルとサブディレクトリを削除
                for filename in os.listdir(selected_dir):
                    file_path = os.path.join(selected_dir, filename)
                    try:
                        if os.path.isfile(file_path) or os.path.islink(file_path):
                            os.remove(file_path)
                            rospy.loginfo(f'ファイルを削除しました: {file_path}')
                        elif os.path.isdir(file_path):
                            shutil.rmtree(file_path)
                            rospy.loginfo(f'Directory deleted: {file_path}')
                    except Exception as e:
                        rospy.logerr(f'ファイル削除中にエラーが発生しました: {file_path}. 詳細: {e}')
                        messagebox.showerror("エラー", f"ファイル削除中にエラーが発生しました: {file_path}\n詳細: {e}")
                        return
            else:
                rospy.loginfo('ユーザがキャンセルを選択しました。')
                return

        # 親ディレクトリを設定
        self.mapping_program.set_base_directory(selected_dir)
        self.status_label.config(text="ステータス: マップ作成完了")
        messagebox.showinfo("情報", "マップ作成が完了しました。")

    def post_processing(self):
        maps_txt_path = filedialog.askopenfilename(
            title="maps.txtを選択",
            filetypes=[("Text Files", "*.txt")],
            defaultextension=".txt"
        )
        if not maps_txt_path:
            messagebox.showerror("エラー", "maps.txtが選択されていません。")
            return

        if os.path.basename(maps_txt_path) != 'maps.txt':
            messagebox.showerror("エラー", "選択されたファイルがmaps.txtではありません。")
            return

        self.status_label.config(text="ステータス: マップ後処理中...")
        self.root.update()

        # ポストプロセッシングを別スレッドで実行
        threading.Thread(target=self.run_post_processing, args=(maps_txt_path,), daemon=True).start()

    def run_post_processing(self, maps_txt_path):
        try:
            self.ros_handler.execute_post_processing(maps_txt_path)
            self.status_label.config(text="ステータス: マップ後処理完了")
        except Exception as e:
            rospy.logerr(f'マップ後処理中にエラーが発生しました: {e}')
            messagebox.showerror("エラー", f"マップ後処理中にエラーが発生しました。\n詳細: {e}")
            self.status_label.config(text="ステータス: エラー発生")

    def param_changed(self, param):
        value = self.ros_param_controller.vars[param].get()
        rospy.loginfo(f'Parameter {param} Changed: {value}')
        self.ros_param_controller.handle_param_change(param, value)

class Application:
    def __init__(self):
        # Initialize Tkinter root first
        self.root = tk.Tk()

        # Initialize Mapping Program
        self.mapping_program = MappingProgram()

        # Initialize ROS Handler
        self.ros_handler = ROSHandler(self.mapping_program)

        # Initialize ROSParamController after root is created
        self.publish_enabled_var = tk.IntVar(value=1)
        self.ros_param_controller = ROSParamController(self.root, self.publish_enabled_var)

        # Initialize GUI
        self.gui = GUI(self.root, self.mapping_program, self.ros_handler, self.ros_param_controller)

        # Start ROS spin in a separate thread
        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

        # Handle graceful shutdown
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        signal.signal(signal.SIGINT, self.signal_handler)

    def ros_spin(self):
        rospy.spin()

    def on_closing(self):
        if messagebox.askokcancel("Quit", "プログラムを終了しますか？"):
            rospy.signal_shutdown("GUIからの終了")
            self.root.destroy()

    def signal_handler(self, sig, frame):
        rospy.loginfo("Shutting down application.")
        rospy.signal_shutdown("Signal received")
        self.root.quit()

    def run(self):
        self.root.mainloop()

if __name__ == '__main__':
    try:
        app = Application()
        app.run()
    except rospy.ROSInterruptException:
        pass
