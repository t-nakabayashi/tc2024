#! /usr/bin/env python3
# coding=utf-8

import rospy
import math
import csv
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Vector3, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import tf
import threading
import os

def quaternion_to_euler(quaternion):
    e = tf.transformations.euler_from_quaternion(
        (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    return Vector3(x=e[0], y=e[1], z=e[2])

# パラメータのデフォルト値を定義
default_waypoint_file = '/home/nakaba/map/waypoint_tsukuba2023.csv'

def get_rosparam(param_name, default):
    try:
        return rospy.get_param(param_name, default)
    except rospy.ROSException:
        return default

# ROSパラメータからウェイポイントファイル名を取得
filename = get_rosparam("/waypoint", default_waypoint_file)
print(f"Waypoint file: {filename}")

num = 0
x_old = 0
y_old = 0
q_old = 0
stop_counter = 0

x_dt = 0
y_dt = 0

# フラグを格納する辞書を追加
flags = {
    'right_is_open': 0,
    'left_is_open': 0,
    'line_is_stop': 0,
    'signal_is_stop': 0,
    'isnot_skipnum': 1  # 初期値は1
}

# 前回のフラグを保存する辞書
previous_flags = flags.copy()

# 変化検出用ロック
flag_lock = threading.Lock()

def update_flags():
    """最新のフラグの状態を表示用に更新"""
    with flag_lock:
        for flag, value in flags.items():
            rospy.loginfo(f"{flag}: {value}")

# 各トピックのコールバック関数
def right_is_open_callback(msg):
    with flag_lock:
        flags['right_is_open'] = msg.data

def left_is_open_callback(msg):
    with flag_lock:
        flags['left_is_open'] = msg.data

def line_is_stop_callback(msg):
    with flag_lock:
        flags['line_is_stop'] = msg.data

def signal_is_stop_callback(msg):
    with flag_lock:
        flags['signal_is_stop'] = msg.data

def isnot_skipnum_callback(msg):
    with flag_lock:
        flags['isnot_skipnum'] = msg.data

def check_parameter_changes(event=None):
    """フラグの変化をチェックし、変化があればウェイポイントを記録する"""
    global previous_flags
    with flag_lock:
        changed = False
        for key in flags:
            if flags[key] != previous_flags[key]:
                changed = True
                break
        if changed:
            record_waypoint(parameter_change=True)
            previous_flags = flags.copy()

def record_waypoint(parameter_change=False):
    """現在の位置とフラグ状態をウェイポイントとして記録する"""
    global num, x_old, y_old, q_old, z_old, orien_z, orien_w

    # 現在の位置とオリエンテーションを取得
    try:
        odom = rospy.wait_for_message('/Odometry', Odometry, timeout=1.0)
    except rospy.ROSException:
        rospy.logerr("Failed to get Odometry message for waypoint recording.")
        return

    orien_z = odom.pose.pose.orientation.z
    orien_w = odom.pose.pose.orientation.w
    x_current = odom.pose.pose.position.x
    y_current = odom.pose.pose.position.y
    z_old = odom.pose.pose.position.z
    q_current = quaternion_to_euler(odom.pose.pose.orientation).z

    # CSVファイルにウェイポイントを書き込む
    try:
        with open(filename, 'a') as f:
            if parameter_change:
                # パラメータ変更によるウェイポイント
                data = f"{num},{x_current},{y_current},{z_old},0,0,{orien_z},{orien_w},"
                data += f"{flags['right_is_open']},{flags['left_is_open']},"
                data += f"{flags['line_is_stop']},{flags['signal_is_stop']},{flags['isnot_skipnum']}\n"
            else:
                # 走行中のウェイポイント
                data = f"{num},{x_current},{y_current},{z_old},0,0,{orien_z},{orien_w},0,0,0,0,{flags['isnot_skipnum']}\n"
            f.write(data)
        rospy.loginfo(f"Waypoint {num} recorded{' due to parameter change' if parameter_change else ''}.")
        num += 1
    except IOError as e:
        rospy.logerr(f"Failed to write to waypoint file: {e}")

def clean_waypoints():
    """ウェイポイントファイルを読み込み、条件に基づいて不要な行を削除"""
    try:
        with open(filename, 'r') as f:
            reader = csv.reader(f)
            waypoints = list(reader)

        # ヘッダーを除いたウェイポイントをチェック
        cleaned_waypoints = [waypoints[0]]  # ヘッダー保持

        i = 1
        while i < len(waypoints) - 1:
            current_wp = waypoints[i]
            
            # 現在のウェイポイントに line_is_stop または signal_is_stop が立っているかどうかをチェック
            if int(current_wp[10]) == 1 or int(current_wp[11]) == 1:
                # line_is_stop または signal_is_stop が立っている行は削除せず、必ず保持
                cleaned_waypoints.append(current_wp)

                # 次のウェイポイントを判定する
                j = i + 1  # 次のウェイポイントを指す
                while j < len(waypoints):
                    next_wp = waypoints[j]
                    
                    # 現在のウェイポイントと次のウェイポイントの距離を計算
                    distance_to_next = math.sqrt(
                        (float(current_wp[1]) - float(next_wp[1])) ** 2 + 
                        (float(current_wp[2]) - float(next_wp[2])) ** 2
                    )
                    
                    # 次のウェイポイントが3m以下かつ、次のウェイポイントに stop フラグが立っていない場合、削除
                    if distance_to_next <= 3.0 and int(next_wp[10]) == 0 and int(next_wp[11]) == 0:
                        rospy.loginfo(f"Deleting waypoint {next_wp[0]} due to proximity to stop waypoint {current_wp[0]}")
                        j += 1  # さらに次のウェイポイントに進む
                    else:
                        break  # 条件を満たさない場合は削除を終了して次のウェイポイントに進む
                
                # その次のウェイポイント（消さない）から処理を続ける
                i = j
            else:
                # line_is_stop または signal_is_stop が立っていないウェイポイントはそのまま追加
                cleaned_waypoints.append(current_wp)
                i += 1

        # 最後のウェイポイントがまだ処理されていない場合、追加
        if i == len(waypoints) - 1:
            cleaned_waypoints.append(waypoints[-1])

     
        # ファイルを上書き保存
        with open(filename, 'w') as f:
            writer = csv.writer(f)
            writer.writerows(cleaned_waypoints)

        rospy.loginfo("Waypoint cleanup complete.")
    except IOError as e:
        rospy.logerr(f"Error reading or writing waypoint file: {e}")

def PoseCallBack(msg):
    global num
    global x_old, y_old, q_old, x_dt, y_dt, stop_counter

    # 停止カウンタが一定以上ならウェイポイントを追加
    if stop_counter > 2:
        stop_counter = -100000
        rospy.loginfo("Stop condition met, recording waypoint.")
        orien_z = msg.pose.pose.orientation.z
        orien_w = msg.pose.pose.orientation.w

        x_old = msg.pose.pose.position.x
        y_old = msg.pose.pose.position.y
        z_old = msg.pose.pose.position.z
        q_old = quaternion_to_euler(msg.pose.pose.orientation).z

        with open(filename, 'a') as f:
            # フラグ情報をCSVに追加
            data = f"{num},{x_old},{y_old},{z_old},0,0,{orien_z},{orien_w},"
            data += f"{flags['right_is_open']},{flags['left_is_open']},{flags['line_is_stop']},{flags['signal_is_stop']},{flags['isnot_skipnum']}\n"
            f.write(data)
        num += 1

    elif (math.sqrt((msg.pose.pose.position.x - x_old) ** 2 + (msg.pose.pose.position.y - y_old) ** 2) > 4 or
          abs(quaternion_to_euler(msg.pose.pose.orientation).z - q_old) > math.pi / 3):
        # ウェイポイントを記録する条件に合致する場合
        orien_z = msg.pose.pose.orientation.z
        orien_w = msg.pose.pose.orientation.w

        x_old = msg.pose.pose.position.x
        y_old = msg.pose.pose.position.y
        z_old = msg.pose.pose.position.z
        q_old = quaternion_to_euler(msg.pose.pose.orientation).z

        with open(filename, 'a') as f:
            # フラグ情報をCSVに追加
            data = f"{num},{x_old},{y_old},{z_old},0,0,{orien_z},{orien_w},0,0,0,0,{flags['isnot_skipnum']}\n"
            f.write(data)
        num += 1

    elif x_dt == msg.pose.pose.position.x and y_dt == msg.pose.pose.position.y:
        stop_counter += 1
    else:
        stop_counter = 0
        x_dt = msg.pose.pose.position.x
        y_dt = msg.pose.pose.position.y

def PoseSub():
    rospy.init_node('pose_sub_reader', anonymous=False)
    
    # Odometryトピックを購読
    rospy.Subscriber('/Odometry', Odometry, PoseCallBack)

    # 各フラグのトピックを購読
    rospy.Subscriber('right_is_open', Int32, right_is_open_callback)
    rospy.Subscriber('left_is_open', Int32, left_is_open_callback)
    rospy.Subscriber('line_is_stop', Int32, line_is_stop_callback)
    rospy.Subscriber('signal_is_stop', Int32, signal_is_stop_callback)
    rospy.Subscriber('isnot_skipnum', Int32, isnot_skipnum_callback)

    # パラメータ監視用のタイマーを設定（0.5秒ごとにチェック）
    rospy.Timer(rospy.Duration(0.5), check_parameter_changes)

    rospy.spin()

if __name__ == '__main__':
    try:
        with open(filename, 'w') as f:
            # CSVヘッダーを修正（フラグの位置を調整）
            header = "num,x,y,z,q1,q2,q3,q4,right_is_open,left_is_open,line_is_stop,signal_is_stop,isnot_skipnum\n"
            f.write(header)
        PoseSub()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception, exiting...")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        clean_waypoints()  # プログラム終了時にウェイポイントのクリーンアップを実行
