#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseActionGoal  # 追加
import math
import threading
import os

class TimeOptimalController:
    def __init__(self):
        # ノードの初期化
        rospy.init_node('time_optimal_controller')

        # パラメータの設定
        self.max_v = rospy.get_param('~MAX_VEL', 1.0)            # 最大線速度 [m/s]
        self.max_w = rospy.get_param('~MAX_W', 1.8)              # 最大角速度 [rad/s]
        self.max_a_v = rospy.get_param('~MAX_ACC_V', 1.0)        # 最大線加速度 [m/s^2]
        self.max_a_w = rospy.get_param('~MAX_ACC_W', 1.5)        # 最大角加速度 [rad/s^2]
        self.position_tolerance = rospy.get_param('~POSITION_TOLERANCE', 0.4)  # 位置誤差の許容範囲 [m]
        self.angle_tolerance = rospy.get_param('~ANGLE_TOLERANCE', 0.2)        # 角度誤差の許容範囲 [rad]
        self.control_rate_hz = rospy.get_param('~CONTROL_RATE', 20.0)          # 制御周期 [Hz]
        self.control_dt = 1.0 / self.control_rate_hz                           # 制御周期 [s]

        # PID制御の初期化（角速度用）
        self.kp_w = 0.6  # Pゲイン
        self.ki_w = 0.02  # Iゲイン
        self.kd_w = 0.02  # Dゲイン
        self.integral_w = 0.0
        self.prev_yaw_error = 0.0
 
        # 障害物検知のパラメータ
        self.robot_width = 0.6  # ロボットの幅 [m]
        self.safety_distance = 0.8  # 障害物との安全距離 [m]
        self.min_obstacle_distance = 0.5  # 最小許容距離(安全距離バッファ) [m]
        self.obstacle_distance = None  # 前方障害物までの距離

        # 現在の状態を保持する変数
        self.current_pose = None
        self.current_velocity = None
        self.current_goal = None

        # 前回のcmd_velを保持
        self.prev_cmd_vel = Twist()

        # ロックの設定（スレッドセーフのため）
        self.lock = threading.Lock()

        # サブスクライバの設定
        rospy.Subscriber('/ypspur_ros/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.goal_callback)  # 修正
        rospy.Subscriber('/scan_livox_front_low_move', LaserScan, self.laser_scan_callback)

        # パブリッシャの設定
        self.cmd_vel_pub = rospy.Publisher('ypspur_ros/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/direction_marker', Marker, queue_size=1)  # マーカーパブリッシャ

        # ログファイルの設定
        self.log_file_path = os.path.join(os.path.expanduser('~'), 'control_log.csv')
        self.log_file = open(self.log_file_path, 'w')
        self.log_file.write('timestamp,v_current,w_current,v_desired,w_desired,angle_diff,distance_error,angle_scaling,v_desired_scaled,obstacle_distance\n')

        # 制御ループの開始
        self.control_loop()

    def __del__(self):
        # ログファイルを閉じる
        self.log_file.close()

    def odom_callback(self, msg):
        """
        Odometryメッセージを受け取って、現在の速度を更新します。
        """
        with self.lock:
            self.current_velocity = msg.twist.twist
 
    def pose_callback(self, msg):
        """
        AMCL Poseメッセージを受け取って、現在の位置姿勢を更新します。
        """
        with self.lock:
            self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        """
        現在のゴールを受け取って、目標位置姿勢を更新します。
        """
        with self.lock: 
            self.current_goal = msg.goal.target_pose.pose  # 修正

    def laser_scan_callback(self, msg):
        """
        LiDARデータを処理し、前方の障害物との最小距離を更新します（ロボットの速度に基づいた動的安全距離）。
        """
        with self.lock:
            # ロボットの幅と前方の最大検出距離
            half_width = self.robot_width / 2.0
            max_detection_distance = 5.0  # 障害物検出の最大距離 [m]

            # 有効な範囲内での並進方向の最小距離を初期化
            x_min = None

            # 各ビームの距離と角度を処理
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment

            for i, r in enumerate(msg.ranges):
                # 距離が無限大またはNaNの場合は無視
                if math.isinf(r) or math.isnan(r) or r < 0.45:
                    continue

                # 各ビームの角度を計算
                angle = angle_min + i * angle_increment

                # 後方のビームの場合は無視
                if math.degrees(angle) < -90.0 or math.degrees(angle) > 90.0:
                    continue

                # ビームの位置をXY平面に変換
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                # 前方の長方形領域内にビームが存在するかチェック
                if 0 < x <= max_detection_distance and -half_width <= y <= half_width:
                    # 障害物が検出された場合、最小距離を更新
                    if x_min is None or x < x_min:
                        x_min = x

            # 更新した障害物までの最小距離を保存
            self.obstacle_distance = x_min

    def quaternion_to_yaw(self, quat):
        """
        クォータニオンからヨー角を計算します。
        """
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle(self, angle):
        """
        角度を -pi から pi の範囲に正規化します。
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def compute_time_optimal_cmd_vel(self, current_pose, current_velocity, target_pose, prev_cmd_vel):
        """
        現在の姿勢と目標の姿勢に基づいて時間最適化されたcmd_velを計算します。
        角度差に応じて速度を減算し、前方の障害物に応じて減速・停止します。

        :param current_pose: geometry_msgs/Pose 現在のロボットの姿勢
        :param current_velocity: geometry_msgs/Twist 現在のロボットの速度
        :param target_pose: geometry_msgs/Pose 目標のロボットの姿勢
        :param prev_cmd_vel: geometry_msgs/Twist 前回のcmd_vel
        :return: geometry_msgs/Twist 新しいcmd_vel, dict デバッグ情報
        """
        # 現在の速度を取得
        v_current = current_velocity.linear.x if current_velocity else 0.0
        w_current = current_velocity.angular.z if current_velocity else 0.0

        # 現在の位置と目標位置の差分
        dx = target_pose.position.x - current_pose.position.x
        dy = target_pose.position.y - current_pose.position.y
        distance_error = math.hypot(dx, dy)

        # 現在の姿勢のヨー角
        current_yaw = self.quaternion_to_yaw(current_pose.orientation)

        # 目標へのベアリング角を計算
        target_bearing = math.atan2(dy, dx)

        # ベアリング角と現在のヨー角の差分を角度誤差とする
        yaw_error = self.normalize_angle(target_bearing - current_yaw)
        angle_diff = abs(yaw_error)

        # 許容誤差内の場合、角度誤差を目標のベクトル向きと現在のヨー角の差分で上書き
        if distance_error <= self.position_tolerance:
            target_yaw = self.quaternion_to_yaw(target_pose.orientation)
            yaw_error = self.normalize_angle(target_yaw - current_yaw)
            angle_diff = abs(yaw_error)

        # PID制御による角速度の計算
        self.integral_w += yaw_error * self.control_dt
        derivative_w = (yaw_error - self.prev_yaw_error) / self.control_dt
        w_desired = (self.kp_w * yaw_error +
                    self.ki_w * self.integral_w +
                    self.kd_w * derivative_w)
        self.prev_yaw_error = yaw_error

        # 角速度の最大制限を適用
        w_desired = max(-self.max_w, min(self.max_w, w_desired))

        # 通常の速度制御（角度差分に応じた加減速）
        if distance_error > 0.0:
            v_desired = min(v_current + self.max_a_v * self.control_dt, self.max_v)
        else:
            v_desired = 0.0

        # 角度差に応じた速度の減速（進行方向が大きくずれている場合に速度を減らす）
        angle_scaling = max(0.0, 1.0 - (angle_diff / math.pi))
        v_desired_scaled = v_desired * angle_scaling

        # 障害物の距離と停止距離に基づいた速度調整
        if self.obstacle_distance is not None:
            max_deceleration = self.max_a_v

            # 停止距離の計算
            stopping_distance = (v_current ** 2) / (2 * max_deceleration) + self.min_obstacle_distance

            # 障害物の距離と停止距離を比較し、速度を減速または停止
            if self.obstacle_distance < stopping_distance:
                rospy.logwarn("Obstacle detected within stopping distance! Distance: {:.2f} m, Stopping Distance: {:.2f} m".format(
                    self.obstacle_distance, stopping_distance
                ))
                v_desired_scaled = 0.0
            elif self.obstacle_distance < self.safety_distance:
                # 安全距離内に障害物がある場合、速度を減速
                v_desired_scaled *= (self.obstacle_distance - self.min_obstacle_distance) / (self.safety_distance - self.min_obstacle_distance)
                v_desired_scaled = max(0.0, v_desired_scaled)

        # 最大速度制限の再確認
        v_desired_scaled = max(0.0, min(self.max_v, v_desired_scaled))

        # 許容誤差内の場合、速度をゼロに設定
        if distance_error <= self.position_tolerance:
            v_desired_scaled = 0.0
            if angle_diff <= self.angle_tolerance:
                w_desired = 0.0
                self.integral_w = 0


        # Twistメッセージの生成
        cmd_vel = Twist()
        cmd_vel.linear.x = v_desired_scaled
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = w_desired

        # デバッグ情報の収集
        debug_info = {
            'v_current': v_current,
            'w_current': w_current,
            'v_desired': v_desired,
            'w_desired': w_desired,
            'angle_diff': angle_diff,
            'distance_error': distance_error,
            'angle_scaling': angle_scaling,
            'v_desired_scaled': v_desired_scaled
        }

        return cmd_vel, debug_info

    def publish_direction_marker(self, current_pose, v_desired, w_desired):
        """
        進行方向を示すマーカーをパブリッシュします。

        :param current_pose: geometry_msgs/Pose 現在のロボットの姿勢
        :param v_desired: float 計算された線速度 [m/s]
        :param w_desired: float 計算された角速度 [rad/s]
        """
        marker = Marker()
        marker.header.frame_id = "map"  # フレームIDを設定（適切なフレームに変更する場合があります）
        marker.header.stamp = rospy.Time.now()
        marker.ns = "direction_marker"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        # 矢印の始点と終点を設定
        start_point = Point()
        start_point.x = current_pose.position.x
        start_point.y = current_pose.position.y
        start_point.z = current_pose.position.z

        # 進行方向を計算
        yaw = self.quaternion_to_yaw(current_pose.orientation)

        # 矢印の長さを設定（速度に比例させることも可能）
        arrow_length = 1.0  # 矢印の長さ [m]

        end_point = Point()
        end_point.x = start_point.x + arrow_length * math.cos(yaw)
        end_point.y = start_point.y + arrow_length * math.sin(yaw)
        end_point.z = start_point.z

        marker.points.append(start_point)
        marker.points.append(end_point)

        # マーカーのスケール設定
        marker.scale.x = 0.1  # 矢印のシャフトの直径
        marker.scale.y = 0.2  # 矢印のヘッドの直径
        marker.scale.z = 0.0  # 矢印のヘッドの長さ

        # マーカーの色設定（RGBA）
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # マーカーの寿命（0の場合は永遠に表示）
        marker.lifetime = rospy.Duration(0)

        # マーカーをパブリッシュ
        self.marker_pub.publish(marker)

    def control_loop(self):
        """
        制御ループを実行し、定期的にcmd_velを発行します。
        """
        rate = rospy.Rate(self.control_rate_hz)
        while not rospy.is_shutdown():
            with self.lock:
                # 必要なデータが揃っているか確認
                if self.current_pose and self.current_velocity and self.current_goal:
                    # 現在の姿勢と目標の姿勢を設定
                    target_pose = self.current_goal

                    # cmd_velを計算
                    cmd_vel, debug_info = self.compute_time_optimal_cmd_vel(
                        current_pose=self.current_pose,
                        current_velocity=self.current_velocity,
                        target_pose=target_pose,
                        prev_cmd_vel=self.prev_cmd_vel
                    )

                    # cmd_velをパブリッシュ
                    self.cmd_vel_pub.publish(cmd_vel)

                    # データを記録
                    timestamp = rospy.Time.now().to_sec()
                    data_line = '{},{},{},{},{},{},{},{},{},{}\n'.format(
                        timestamp,
                        debug_info['v_current'],
                        debug_info['w_current'],
                        debug_info['v_desired'],
                        debug_info['w_desired'],
                        debug_info['angle_diff'],
                        debug_info['distance_error'],
                        debug_info['angle_scaling'],
                        debug_info['v_desired_scaled'],
                        self.obstacle_distance if self.obstacle_distance is not None else 'NaN'
                    )
                    self.log_file.write(data_line)
                    self.log_file.flush()

                    # 進行方向マーカーをパブリッシュ
                    self.publish_direction_marker(self.current_pose, cmd_vel.linear.x, cmd_vel.angular.z)

                    # 前回のcmd_velを更新
                    self.prev_cmd_vel = cmd_vel
                else:
                    # 足りないデータを報告
                    if not self.current_pose:
                        rospy.logwarn_throttle(5, "Waiting for AMCL pose data.")
                    if not self.current_velocity:
                        rospy.logwarn_throttle(5, "Waiting for odometry data.")
                    if not self.current_goal:
                        rospy.logwarn_throttle(5, "Waiting for goal data.")

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = TimeOptimalController()
    except rospy.ROSInterruptException:
        pass