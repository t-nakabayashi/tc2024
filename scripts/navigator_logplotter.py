import pandas as pd
import matplotlib.pyplot as plt

# データの読み込み
log_file_path = '/home/your_username/control_log.csv'  # ログファイルのパスを指定
df = pd.read_csv(log_file_path)

# タイムスタンプを0からの相対時間に変換
df['relative_time'] = df['timestamp'] - df['timestamp'][0]

# プロットの設定
plt.figure(figsize=(12, 8))

# サブプロット1: 速度と目標速度
plt.subplot(2, 1, 1)
plt.plot(df['relative_time'], df['v_current'], label='Current Velocity')
plt.plot(df['relative_time'], df['v_desired_scaled'], label='Desired Velocity')
plt.xlabel('Time [s]')
plt.ylabel('Linear Velocity [m/s]')
plt.title('Linear Velocity')
plt.legend()
plt.grid(True)

# サブプロット2: 角速度と目標角速度
plt.subplot(2, 1, 2)
plt.plot(df['relative_time'], df['w_current'], label='Current Angular Velocity')
plt.plot(df['relative_time'], df['w_desired'], label='Desired Angular Velocity')
plt.xlabel('Time [s]')
plt.ylabel('Angular Velocity [rad/s]')
plt.title('Angular Velocity')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()
