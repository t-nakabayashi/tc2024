#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseArray
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import time

# 広がりと標準偏差を保持するリスト
time_data = []
spread_data = []
spread_history = []  # 標準偏差の履歴を保存

# 開始時刻を保持する変数
start_time = None

# コールバック関数: トピックからデータを受け取った時に呼ばれる
def callback(pose_array):
    global time_data, spread_data, spread_history, start_time

    # パーティクルのx座標を取得
    x_positions = [pose.position.x for pose in pose_array.poses]

    # x座標の標準偏差（広がり）を計算
    spread = np.std(x_positions)
    
    # 現在の時間を取得
    if start_time is None:
        start_time = time.time()
    current_time = time.time() - start_time

    # 時間と広がりデータを追加
    time_data.append(current_time)
    spread_data.append(spread)
    spread_history.append(spread)  # 標準偏差の履歴を保存

    # 平均と分散を計算
    spread_mean = np.mean(spread_history)
    spread_variance = np.var(spread_history)

    # 平均と分散をターミナルに表示
    rospy.loginfo(f"Spread Mean: {spread_mean:.4f}, Spread Variance: {spread_variance:.4f}")

# ROSノードの初期化
def listener():
    rospy.init_node('particle_spread_visualizer', anonymous=True)
    
    # トピック購読
    rospy.Subscriber("/mcl_3dl/particles", PoseArray, callback)

    # 可視化処理の初期化
    fig, ax = plt.subplots()

    # グラフの設定
    ax.set_title('Particle Spread Over Time (1D)')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Spread (Std Dev of X positions)')

    # 更新関数：グラフをリアルタイムに更新
    def update(frame):
        ax.clear()  # クリアして再描画
        ax.plot(time_data, spread_data, '-o')
        ax.set_title('Particle Spread Over Time (1D)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Spread (Std Dev of X positions)')
        ax.relim()  # 軸の範囲をデータに基づいて自動スケーリング
        ax.autoscale_view()

    # アニメーションの設定
    ani = FuncAnimation(fig, update, interval=100)

    # グラフ表示
    plt.show()

    # ROSのスピン（購読を維持）
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
