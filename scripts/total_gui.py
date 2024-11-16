#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import tkinter as tk
import subprocess
import time
import signal
import atexit  # プログラム終了時の処理を追加

# 各launchファイルの情報
LOCARIZATION_LAUNCH = ["roslaunch", "tc2024", "3-1_locarization_fastlio.launch"]
NAVIGATION_LAUNCH_TEMPLATE = ["roslaunch", "tc2024", "4_run.launch", "start_num:="]
LOCARIZATION_NEXT_LAUNCH = ["roslaunch", "tc2024", "3-1_locarization_fastlio_next.launch"]
NAVIGATION_NEXT_LAUNCH = ["roslaunch", "tc2024", "4_run_next.launch"]

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
            process.send_signal(signal.SIGINT)  # SIGINTを送信
            try:
                process.wait(timeout=5)  # 正常終了を待つ
            except subprocess.TimeoutExpired:
                print(f"{name} did not stop in time. Killing it.")
                process.kill()  # 終了しない場合は強制終了
    print("All launches stopped.")

# プログラム終了時にすべてのlaunchファイルを停止
atexit.register(stop_all_launches)

def wait_for_all_stopped(names):
    """全てのプロセスが停止するまで待機"""
    for name in names:
        if name in processes:
            while processes[name].poll() is None:
                print(f"Waiting for {name} to stop...")
                time.sleep(1)

def start_locarization():
    start_launch("locarization", LOCARIZATION_LAUNCH)

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

def stop_navigation():
    stop_launch("navigation")

def change_map():
    # ローカライゼーションとナビゲーションの停止
    stop_launch("locarization")
    stop_launch("navigation")

    # 両方の停止を確認
    wait_for_all_stopped(["locarization", "navigation"])

    # 次のマップ用のlaunchファイルを起動
    start_launch("locarization_next", LOCARIZATION_NEXT_LAUNCH)
    start_launch("navigation_next", NAVIGATION_NEXT_LAUNCH)

# GUIの構築
root = tk.Tk()
root.title("ROS Launch Manager")
root.geometry("500x300")  # ウィンドウサイズを少し広く設定

# GUIを常に最前面に設定
root.attributes("-topmost", True)

# ボタン1: Locarization
locarization_frame = tk.Frame(root)
locarization_frame.pack(pady=10, anchor="w")
locarization_button = tk.Button(locarization_frame, text="Start Locarization", command=start_locarization, width=20)
locarization_button.pack(side="left", padx=5)
stop_locarization_button = tk.Button(locarization_frame, text="Stop Locarization", command=stop_locarization, width=20)
stop_locarization_button.pack(side="left", padx=5)

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
navigation_button = tk.Button(navigation_frame, text="Start Navigation", command=start_navigation_with_waypoint, width=20)
navigation_button.pack(side="left", padx=5)
stop_navigation_button = tk.Button(navigation_frame, text="Stop Navigation", command=stop_navigation, width=20)
stop_navigation_button.pack(side="left", padx=5)

# ボタン3: Change Map
change_map_frame = tk.Frame(root)
change_map_frame.pack(pady=10, anchor="w")
change_map_button = tk.Button(change_map_frame, text="Change Map", command=change_map, width=20)
change_map_button.pack(side="left", padx=5)

# プログラム終了時のフック（GUIが閉じられた際にも呼ばれる）
root.protocol("WM_DELETE_WINDOW", lambda: (stop_all_launches(), root.destroy()))

# メインループ
root.mainloop()
