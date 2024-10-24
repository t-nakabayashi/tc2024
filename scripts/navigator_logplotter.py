import pandas as pd
import matplotlib.pyplot as plt

# データの読み込み
log_file_path = '~/control_log.csv'  # ログファイルのパスを指定
df = pd.read_csv(log_file_path)

# タイムスタンプを0からの相対時間に変換
df['relative_time'] = df['timestamp'] - df['timestamp'][0]

# グラフの作成
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

# サブプロット1: 距離誤差と目標速度
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Distance Error', color='tab:blue')
ax1.plot(df['relative_time'], df['distance_error'], label='distance_error', color='tab:blue')
ax1.tick_params(axis='y', labelcolor='tab:blue')
ax1.grid(True)

# 右側のY軸（v_desired_scaled）
ax1b = ax1.twinx()
ax1b.set_ylabel('v_desired_scaled [m/s]', color='tab:red')
ax1b.plot(df['relative_time'], df['v_desired'], label='v_desired', color='tab:red', linestyle='--')
ax1b.tick_params(axis='y', labelcolor='tab:red')

# サブプロット2: 角度差と目標速度
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Angle Difference [rad]', color='tab:green')
ax2.plot(df['relative_time'], df['w_current'], label='w_current', color='tab:green')
ax2.tick_params(axis='y', labelcolor='tab:green')
ax2.grid(True)

# 右側のY軸（v_desired_scaled）
ax2b = ax2.twinx()
ax2b.set_ylabel('v_desired_scaled [m/s]', color='tab:red')
ax2b.plot(df['relative_time'], df['w_desired'], label='w_desired', color='tab:red', linestyle='--')
ax2b.tick_params(axis='y', labelcolor='tab:red')

plt.tight_layout()
plt.show()
