#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
import math
import threading
import os

class TimeOptimalController:
    def __init__(self):
        # ノードの初期化
        rospy.init_node('time_optimal_controller')

        # パラメータの設定
        self.max_v = rospy.get_param('~MAX_VEL', 0.6)            # 最大線速度 [m/s]
        self.max_w = rospy.get_param('~MAX_W', 1.5)              # 最大角速度 [rad/s]
        self.max_a_v = rospy.get_param('~MAX_ACC_V', 1.0)        # 最大線加速度 [m/s^2]
        self.max_a_w = rospy.get_param('~MAX_ACC_W', 0.8)        # 最大角加速度 [rad/s^2]
        self.position_tolerance = rospy.get_param('~POSITION_TOLERANCE', 0.3)  # 位置誤差の許容範囲 [m]
        self.angle_tolerance = rospy.get_param('~ANGLE_TOLERANCE', 0.2)        # 角度誤差の許容範囲 [rad]
        self.control_rate_hz = rospy.get_param('~CONTROL_RATE', 20.0)          # 制御周期 [Hz]
        self.control_dt = 1.0 / self.control_rate_hz                           # 制御周期 [s]

        # 障害物検知のパラメータ
        self.robot_width = 0.8  # ロボットの幅 [m]
        self.safety_distance = 1.5  # 障害物との安全距離 [m]
        self.min_obstacle_distance = 0.6  # 最小許容距離 [m]
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
        rospy.Subscriber('/move_base/current_goal', PoseStamped, self.goal_callback)
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
            self.current_goal = msg.pose

    def laser_scan_callback(self, msg):
        """
        LiDARデータを処理し、前方の障害物との最小距離を更新します。
        """
        with self.lock:
            # ロボット前方±half_widthの範囲で障害物を検出
            half_width = self.robot_width / 2.0
            max_detection_distance = 5.0  # 障害物検出の最大距離 [m]
            angle_width = math.atan2(half_width, max_detection_distance)  # ラジアン

            # インデックスの計算
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            num_ranges = len(msg.ranges)

            # 正面のインデックス
            index_center = int(round((0.0 - angle_min) / angle_increment))

            # 左右のインデックス
            index_left = int(round((angle_width - angle_min) / angle_increment))
            index_right = int(round((-angle_width - angle_min) / angle_increment))

            # インデックスの範囲をチェック
            index_left = max(min(index_left, num_ranges - 1), 0)
            index_right = max(min(index_right, num_ranges - 1), 0)

            # インデックスの順序を確認
            if index_left < index_right:
                index_left, index_right = index_right, index_left

            # 範囲内の距離データを取得
            ranges_ahead = msg.ranges[index_right:index_left+1]

            # 有効な距離データをフィルタリング
            valid_ranges = [r for r in ranges_ahead if not math.isinf(r) and not math.isnan(r) and r > 0.004]

            if valid_ranges:
                self.obstacle_distance = min(valid_ranges)
            else:
                self.obstacle_distance = None

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

        # 位置誤差と角度誤差が許容範囲内かどうかをチェック
        position_within_tolerance = distance_error <= self.position_tolerance

        # 角度許容範囲のチェック
        angle_within_tolerance = abs(yaw_error) <= self.angle_tolerance

        # 角度差分に応じて速度を減算（角度差が大きいと速度を減らす）
        if distance_error >= 1:
            angle_scaling = max(0.0, min(1.0, (math.pi - angle_diff) / math.pi))
        else:
            angle_scaling = 1.0

        # 線速度の計算
        if not position_within_tolerance and distance_error > 0.0:
            # 停止するために必要な減速距離
            stopping_distance = (v_current ** 2) / (2 * self.max_a_v) if self.max_a_v != 0 else 0.0

            if distance_error > stopping_distance:
                # 加速
                v_desired = v_current + self.max_a_v * self.control_dt
                v_desired = min(v_desired, self.max_v)
            else:
                # 減速
                v_desired = v_current - self.max_a_v * self.control_dt
                v_desired = max(v_desired, 0.0)
        else:
            # 許容範囲内の場合、線速度をゼロに設定
            v_desired = 0.0

        # 角度差分に応じて速度を減算
        v_desired_scaled = v_desired * angle_scaling

        # 角速度の計算
        if not angle_within_tolerance and abs(yaw_error) > 0.01:
            # 停止するために必要な減速角度
            stopping_angle = (w_current ** 2) / (2 * self.max_a_w) if self.max_a_w != 0 else 0.0

            if abs(yaw_error) > stopping_angle:
                # 加速
                if yaw_error > 0:
                    w_desired = w_current + self.max_a_w * self.control_dt
                    w_desired = min(w_desired, self.max_w)
                else:
                    w_desired = w_current - self.max_a_w * self.control_dt
                    w_desired = max(w_desired, -self.max_w)
            else:
                # 減速
                if w_current > 0:
                    w_desired = w_current - self.max_a_w * self.control_dt
                    w_desired = max(w_desired, 0.0)
                else:
                    w_desired = w_current + self.max_a_w * self.control_dt
                    w_desired = min(w_desired, 0.0)
        else:
            # 許容範囲内の場合、角速度をゼロに設定
            w_desired = 0.0

        # 前方の障害物に応じて速度を調整
        if self.obstacle_distance is not None:
            if self.obstacle_distance < self.min_obstacle_distance:
                # 障害物が近すぎる場合は停止
                v_desired_scaled = 0.0
            elif self.obstacle_distance < self.safety_distance:
                # 安全距離内の場合、速度を減速
                v_desired_scaled *= (self.obstacle_distance - self.min_obstacle_distance) / (self.safety_distance - self.min_obstacle_distance)
                v_desired_scaled = max(0.0, v_desired_scaled)
            # 障害物が十分遠い場合はそのまま

        # 最大速度制限の再確認
        v_desired_scaled = max(0.0, min(self.max_v, v_desired_scaled))
        w_desired = max(-self.max_w, min(self.max_w, w_desired))

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
