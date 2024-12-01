#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import tkinter as tk
import subprocess
import time
import signal
import threading
import atexit
import rospy  # 追加：ROSパッケージのインポート
from std_msgs.msg import Int32  # 追加：メッセージ型のインポート

# 各launchファイルの情報
LOCARIZATION_LAUNCH = ["roslaunch", "tc2024", "3-1_locarization_fastlio_1st.launch"]
NAVIGATION_LAUNCH_TEMPLATE = ["roslaunch", "tc2024", "4_run_1st.launch", "start_num:="]
LOCARIZATION_NEXT_LAUNCH = ["roslaunch", "tc2024", "3-1_locarization_fastlio_next.launch"]
NAVIGATION_NEXT_LAUNCH = ["roslaunch", "tc2024", "4_run_next.launch", "start_num:="]

# プロセスを保持する辞書
processes = {}

def start_launch(name, command):
    """Launchファイルを起動"""
    if name in processes and processes[name].poll() is None:
        print(f"{name} is already running.")
        return
    print(f"Starting {name}...")
    processes[name] = subprocess.Popen(command)

def stop_launch(name):
    """Launchファイルを停止"""
    if name in processes and processes[name].poll() is None:
        print(f"Stopping {name}...")
        processes[name].send_signal(signal.SIGINT)
        processes[name].wait()
        print(f"{name} stopped.")
    else:
        print(f"{name} is not running or already stopped.")

def stop_all_launches():
    """全てのLaunchファイルを強制終了"""
    print("Stopping all launches...")
    for name, process in processes.items():
        if process.poll() is None:
            print(f"Force stopping {name}...")
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)  # 正常終了を待つ
            except subprocess.TimeoutExpired:
                print(f"{name} did not stop in time. Killing it.")
                process.kill()  # 終了しない場合は強制終了
    print("All launches stopped.")

# プログラム終了時にすべてのlaunchファイルを停止
atexit.register(stop_all_launches)

def start_locarization():
    start_launch("locarization", LOCARIZATION_LAUNCH)

def restart_locarization():
    """ローカライゼーションの再起動"""
    stop_launch("locarization")
    start_launch("locarization", LOCARIZATION_NEXT_LAUNCH)

def stop_locarization():
    stop_launch("locarization")

def start_navigation_with_waypoint():
    """ウェイポイント番号を指定してナビゲーションを起動"""
    waypoint = waypoint_entry.get() or "0"  # デフォルト値は0
    if not waypoint.isdigit():
        print("Invalid waypoint number. Please enter a numeric value.")
        return
    # start_num:=<waypoint> を1つの文字列として扱う
    navigation_command = NAVIGATION_LAUNCH_TEMPLATE + [f"start_num:={waypoint}"]
    start_launch("navigation", navigation_command)

def restart_navigation():
    """ナビゲーションの再起動"""
    stop_launch("navigation")
    waypoint = "1"  # デフォルト値は0
    if not waypoint.isdigit():
        print("Invalid waypoint number. Please enter a numeric value.")
        return
    # start_num:=<waypoint> を1つの文字列として扱う
    navigation_command = NAVIGATION_NEXT_LAUNCH + [f"start_num:={waypoint}"]
    start_launch("navigation", navigation_command)

def stop_navigation():
    stop_launch("navigation")

def condition_based_restart():
    """特定条件に基づいて再起動を実行"""
    while True:
        if check_condition_locarization():
            print("Condition met for locarization restart.")
            restart_locarization()
        if check_condition_navigation():
            print("Condition met for navigation restart.")
            restart_navigation()
        time.sleep(5)  # 条件を5秒間隔でチェック

def check_condition_locarization():
    """ローカライゼーション再起動の条件チェック（ダミー実装）"""
    # 条件ロジックをここに実装
    return False  # 常にFalse（条件が満たされる場合のみTrueに変更）

def check_condition_navigation():
    """ナビゲーション再起動の条件チェック（ダミー実装）"""
    # 条件ロジックをここに実装
    return False  # 常にFalse（条件が満たされる場合のみTrueに変更）

# 条件チェックをバックグラウンドで実行
threading.Thread(target=condition_based_restart, daemon=True).start()

# 追加：ROSノードの初期化（disable_signals=Trueでシグナルを無効化）
rospy.init_node('gui_node', anonymous=True, disable_signals=True)

# 追加：パブリッシャーの作成
manual_start_pub = rospy.Publisher('/manual_start', Int32, queue_size=1)

def send_manual_start():
    """手動再開のメッセージを送信"""
    manual_start_pub.publish(1)
    print("Manual start signal sent.")

# GUIの構築
root = tk.Tk()
root.title("ROS Launch Manager")
root.geometry("600x300")  # ウィンドウサイズを少し広く設定

# GUIを常に最前面に設定
root.attributes("-topmost", True)

# ボタン1: Locarization
locarization_frame = tk.Frame(root)
locarization_frame.pack(pady=10, anchor="w")
start_locarization_button = tk.Button(locarization_frame, text="Start Locarization", command=start_locarization, width=20)
start_locarization_button.pack(side="left", padx=5)
stop_locarization_button = tk.Button(locarization_frame, text="Stop Locarization", command=stop_locarization, width=20)
stop_locarization_button.pack(side="left", padx=5)
restart_locarization_button = tk.Button(locarization_frame, text="Restart Locarization", command=restart_locarization, width=20)
restart_locarization_button.pack(side="left", padx=5)

# ウェイポイント番号入力
waypoint_frame = tk.Frame(root)
waypoint_frame.pack(pady=10, anchor="w")
waypoint_label = tk.Label(waypoint_frame, text="Waypoint (default: 0):", width=20, anchor="w")
waypoint_label.pack(side="left")
waypoint_entry = tk.Entry(waypoint_frame)
waypoint_entry.insert(0, "0")  # デフォルト値を0に設定
waypoint_entry.pack(side="left", padx=5)

# ボタン2: Navigation
navigation_frame = tk.Frame(root)
navigation_frame.pack(pady=10, anchor="w")
start_navigation_button = tk.Button(navigation_frame, text="Start Navigation", command=start_navigation_with_waypoint, width=20)
start_navigation_button.pack(side="left", padx=5)
stop_navigation_button = tk.Button(navigation_frame, text="Stop Navigation", command=stop_navigation, width=20)
stop_navigation_button.pack(side="left", padx=5)
restart_navigation_button = tk.Button(navigation_frame, text="Restart Navigation", command=restart_navigation, width=20)
restart_navigation_button.pack(side="left", padx=5)

# 追加：手動再開ボタンの追加
manual_start_frame = tk.Frame(root)
manual_start_frame.pack(pady=10, anchor="w")
manual_start_button = tk.Button(manual_start_frame, text="Manual Start", command=send_manual_start, width=20, bg='green', fg='white')
manual_start_button.pack(side="left", padx=5)

# プログラム終了時のフック（GUIが閉じられた際にも呼ばれる）
root.protocol("WM_DELETE_WINDOW", lambda: (stop_all_launches(), root.destroy()))

# メインループ
root.mainloop()
