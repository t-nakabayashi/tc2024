#!/usr/bin/env python3

import rospy
import actionlib
import tf
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray  # 修正: GoalStatus から GoalStatusArray へ
import csv
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
import random
from std_msgs.msg import Int32
import numpy as np
from sensor_msgs.msg import LaserScan
import cv2

# List of waypoints to navigate
waypoints = []
# Flag to indicate if there is an error reaching the goal
isGoalError = False
# Robot's current pose coordinates
pose_x = 0
pose_y = 0
# Flags for stop and boost control
stopFlag = 0
boostFlag = 0
# Flag to indicate if robot is moving towards a sub-goal
isSubGoalActive = False
# List of pointcloud data
pointcloud = np.empty((0, 2))
# offset for obstacle avoidance
offset_l = 0.0
offset_r = 0.0

# 手動再開フラグの初期化
manual_start_flag = 0

# gpt outputの初期化
gpt_output = 99

# Publishers to control robot velocity and initialize position
pub_vel = rospy.Publisher("/ypspur_ros/cmd_vel", Twist, queue_size=10)
pub_init = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)

def quaternion_to_euler(q1, q2, q3, q4):
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).

    Args:
        q1, q2, q3, q4: Quaternion components.

    Returns:
        geometry_msgs/Vector3: Vector containing roll, pitch, yaw.
    """
    e = tf.transformations.euler_from_quaternion((q1, q2, q3, q4))
    return Vector3(x=e[0], y=e[1], z=e[2])

def goal_pose(pose):
    """
    Create a MoveBaseGoal from the provided pose information.

    Args:
        pose: Tuple containing position and orientation information.

    Returns:
        MoveBaseGoal: Goal for the robot to move to.
    """
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    return goal_pose

def goalstatusCallBack(data):
    """
    Callback function to monitor the goal status.
    Checks if the goal was not successful (e.g., aborted or failed).

    Args:
        data: GoalStatusArray message.
    """
    global isGoalError
    isGoalError = False
    for status in data.status_list:
        if status.status >= 4:  # Status >= 4 indicates failure (e.g., ABORTED, REJECTED)
            isGoalError = True

def velCallBack(data):
    """
    ジョイスティック制御の名残 TODO削除
    """

    pub_vel.publish(data)


def mclposeCallBack(data):
    """
    Callback function to update the robot's current position from localization.

    Args:
        data: PoseWithCovarianceStamped message with current pose information.
    """
    global pose_x, pose_y
    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y

def laserScanViewer(window_name, points, vline_l, vline_r, robot_width, half_width, max_obstacle_distance):
    # 画面表示用画像の生成
    map_range = 8.0 #[m]
    x_min = 0.0
    x_max = (map_range/2) * 2
    y_min = -1.0 * (map_range/2)
    y_max = (map_range/2)
    pixel_pitch = 100 #[pix/m]
    width = int((y_max - y_min) * pixel_pitch) #[pix]
    height = int((x_max - x_min) * pixel_pitch) #[pix]
    grid_map = np.zeros((height, width, 3))
    grid_map += 255

    pix_x = int(width / 2) - int(robot_width * pixel_pitch)
    cv2.line(grid_map, (pix_x, height), (pix_x, 0), (0, 0, 0), thickness=2, lineType=cv2.LINE_AA)
    pix_x = int(width / 2) + int(robot_width * pixel_pitch)
    cv2.line(grid_map, (pix_x, height), (pix_x, 0), (0, 0, 0), thickness=2, lineType=cv2.LINE_AA)
    pix_y = height - int(max_obstacle_distance * pixel_pitch)
    cv2.line(grid_map, (0, pix_y), (width, pix_y), (0, 0, 0), thickness=2, lineType=cv2.LINE_AA)
    for i in range(len(points)):
        x = points[i][0]
        y = points[i][1]
        if x_min < x < x_max and y_min < y < y_max:
            pix_x = int(width / 2) - int(y * pixel_pitch)
            pix_y = height - int(x * pixel_pitch)
            if 0 <= pix_x < width and 0 <= pix_y < height:
                cv2.circle(grid_map, (pix_x, pix_y), 4, (0, 0, 255), -1)
    pix_x = int(width / 2) - int(vline_l * pixel_pitch)
    if vline_l != 0.0 and 0 <= pix_x:
        cv2.line(grid_map, (pix_x, height), (pix_x, 0), (255, 0, 0), thickness=2, lineType=cv2.LINE_AA)
    pix_x = int(width / 2) - int(vline_r * pixel_pitch)
    if vline_r != 0.0 and 0 <= pix_x:
        cv2.line(grid_map, (pix_x, height), (pix_x, 0), (255, 0, 0), thickness=2, lineType=cv2.LINE_AA)
    grid_map_show = cv2.resize(grid_map, (500,500))
    cv2.imshow(window_name, grid_map_show)
    cv2.waitKey(1)

def calcAvoidanceOffset(xy, robot_width, half_width):
    # 始点をx軸上(y=0)として、始点に近い方から順に隣の点とのy方向の距離を算出し障害物の端を探す
    offset = y = y_prev = 0.0
    for i in range(len(xy)):
        y = abs(xy[i][1])
        # 隣の点との間がロボットの幅+車幅半分のマージンよりも開いている場合
        if y - y_prev > (robot_width + half_width):
            # 車幅半分のマージンを設けてオフセットを算出
            offset = y_prev + (robot_width + half_width)/2.0
            # ロボットの直進経路上(左/右半分)に障害物がない場合
            if y_prev == 0.0:
                # 避ける必要なし
                #offset = 0.0
                break
            if offset < 3.0:
                # 前方の障害物を回避するためのオフセット距離として採用
                break
        # 外側に点が存在しない場合
        elif i == len(xy) - 1:
            offset = y + (robot_width + half_width)/2.0
            if offset < 3.0:
                # 前方の障害物を回避するためのオフセット距離として採用
                break
        y_prev = y
    return offset

def laserScanCallback(data):
    """
    LiDARデータを処理し、前方の障害物を回避するための左右の最小オフセット距離を更新します。
    """
    global offset_l,offset_r

    # 各ビームの距離と角度を処理
    angle_min = data.angle_min
    angle_increment = data.angle_increment

    xy = np.empty((0, 2))
    for i, r in enumerate(data.ranges):
        # 距離が無限大またはNaNの場合は無視
        if math.isinf(r) or math.isnan(r) or r < 0.2:
            continue

        # 各ビームの角度を計算
        angle = angle_min + i * angle_increment

        # 後方のビームの場合は無視
        if math.degrees(angle) < -90.0 or math.degrees(angle) > 90.0:
            continue

        # ビームの位置をXY平面に変換
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        xy = np.append(xy, np.array([[x,y]]), axis=0)

    # 参照用のグローバル変数に点群情報をコピー(本当は排他した方が良い)
    pointcloud = xy.copy()

    # ロボットの幅 [m]
    robot_width = 0.6  
    half_width = robot_width / 2.0
    # 回避対象障害物の最大距離 [m]
    max_obstacle_distance = 1.5

    # 点群データがある場合
    if len(xy) > 0:
        # x方向の距離から回避対象の障害物を抽出
        xy_extract = xy[xy[:,0] <= max_obstacle_distance]
        # x軸を境に左右の点群に分割
        xy_extract_l = xy_extract[xy_extract[:,1] > 0.0]
        xy_extract_r = xy_extract[xy_extract[:,1] < 0.0]
        # 左右の点群をy方向の距離でソート
        xy_extract_l = sorted(xy_extract_l, key=lambda c: c[1])
        xy_extract_r = sorted(xy_extract_r, key=lambda c: c[1], reverse=True)
        # 始点をx軸上(y=0)として、始点に近い方から順に隣の点とのy方向の距離を算出し障害物の端を探す
        offset = calcAvoidanceOffset(xy_extract_l, robot_width, half_width)
        if offset <= 1.0:
            offset_l = offset
        else:
            offset_l = 1.0

        offset = calcAvoidanceOffset(xy_extract_r, robot_width, half_width)
        if offset <= 1.0:
            offset_r = offset
        else:
            offset_r = 1.0
            
    #print("offset_l", offset_l, "offset_r", offset_r)
    #laserScanViewer("laserscan", pointcloud, offset_l, -1.0 * offset_r, robot_width, half_width, max_obstacle_distance)

# 追加：手動再開のコールバック関数
def manual_start_callback(data):
    global manual_start_flag
    manual_start_flag = data.data  # データをフラグに格納

# GPT結果格納のコールバック関数
def gpt_output_callback(data):
    global gpt_output # 不定：９９、障害物がない：０、人やロボット：１、カラーコーン：２
    gpt_putput = data.data 
    print(gpt_output)

if __name__ == '__main__':
    rospy.init_node('patrol')  # Initialize the patrol node

    # Publisher for recognizing signals
    pub_recog = rospy.Publisher('recog_flag', Int32, queue_size=1)

    # Initial values for controlling timing
    waitCounter_ms = 0
    waitTime_ms = 50
    # Get parameters for starting waypoint and waypoint file
    start_num = rospy.get_param("~start_num", 0)
    waypoint_name = rospy.get_param("~waypoint", '/home/nakaba/map/waypoint_tsukuba2023.csv')


    # Set up the action client for move_base
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Subscribe to relevant topics
    rospy.Subscriber('/move_base/status', GoalStatusArray, goalstatusCallBack)  # 修正: GoalStatusArray を使用
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, mclposeCallBack)
    rospy.Subscriber('/ypspur_ros/cmd_vel_nav', Twist, velCallBack)
    rospy.Subscriber('/scan_livox_front_low_move', LaserScan, laserScanCallback)
    # 追加：手動再開トピックの購読
    rospy.Subscriber('/manual_start', Int32, manual_start_callback)

    # GPT認識モジュールの購読
    rospy.Subscriber('/gpt_output', Int32, gpt_output_callback)

    # Read the waypoints from the CSV file
    with open(waypoint_name, 'r') as f:
        counter = 0
        reader = csv.reader(f)
        header = next(reader)  # Skip the header row

        for row in reader:
            if counter < start_num:
                counter += 1
            else:
                # Append waypoint data in a structured manner
                waypoints.append([
                    (float(row[1]), float(row[2]), 0.0),  # Position (x, y, z)
                    (0.0, 0.0, float(row[6]), float(row[7])),  # Orientation (quaternion)
                    (float(row[8]), float(row[9]), int(row[10]), int(row[11]), int(row[12])),  # Flags for conditions
                    (int(row[0]))  # Waypoint number
                ])

    manual_start_flag = 0  # 手動再開フラグをリセット
    while manual_start_flag != 1 and not rospy.is_shutdown():
        rospy.sleep(0.1)

    # Main loop for patrolling through waypoints
    while not rospy.is_shutdown():
        for pose_index, pose in enumerate(waypoints):  # enumerateを使用してpose_indexを取得
            boostFlag = 0
            goal = goal_pose(pose)
            client.send_goal(goal)  # Send the goal to move_base
            isSubGoalActive = False  # Initially, no sub-goal is active

            # Display current waypoint information
            status_string = "num:" + str(pose[3])
            pose_string = (" right_open ", " left_open ", " line_stop ", " signal_stop ", " not_skip ")
            for i, p in enumerate(pose[2]):
                if p != 0:
                    status_string += pose_string[i]
                    if i <= 1:
                        status_string += str(p) + "m,"
            print(status_string)

            original_goal = goal_pose(pose)  # Store the original goal

            # Reset manual start flag
            manual_start_flag = 0  # 手動再開フラグをリセット

            # Wait until the goal is reached or some condition is met
            while not rospy.is_shutdown():
                if int(pose[3]) < 1:
                    break

                # Calculate the distance to the current goal
                distance_to_goal = math.sqrt((pose_x - goal.target_pose.pose.position.x) ** 2 + (pose_y - goal.target_pose.pose.position.y) ** 2)
                
                if distance_to_goal <= 0.55 and isSubGoalActive != True:  # Close enough to the goal
                    if not pose[2][2] and not pose[2][3]:
                        # If both stop flags are False, move to the next goal immediately
                        print("next")
                        waitCounter_ms = 0
                        break
                    elif pose[2][2]:
                        waitCounter_ms += waitTime_ms
                        # Line stop flag is True
                        if distance_to_goal <= 0.6:
                            stopFlag = 1
                            print("wait!!")
                            # input()待ちの処理を手動再開待ちに変更
                            while manual_start_flag != 1 and not rospy.is_shutdown():
                                rospy.sleep(0.1)
                            stopFlag = 0
                            manual_start_flag = 0  # フラグをリセット
                            waitCounter_ms = 0
                            break
                    elif pose[2][3]:
                        # Signal stop flag is True
                        if distance_to_goal <= 0.6:
                            print("signal!!!")
                            stopFlag = 1
                            recog = 0
                            while recog != 1 and manual_start_flag != 1 and not rospy.is_shutdown():
                                pub_recog.publish(1)
                                try:
                                    recog_data = rospy.wait_for_message('sig_recog', Int32, timeout=1)
                                    recog = recog_data.data
                                    print(recog)
                                except KeyboardInterrupt:
                                    break
                                except:
                                    print("sig_recog didn't come....")
                                    recog = 0
                                rospy.sleep(0.1)  

                            stopFlag = 0
                            manual_start_flag = 0  # フラグをリセット
                            waitCounter_ms = 0
                            break
                else:
                    # Update waiting status and re-send goal if necessary
                    pub_recog.publish(0)
                    waitCounter_ms += waitTime_ms
                    if isGoalError and not isSubGoalActive:
                        # Re-send original goal if there was an error and no sub-goal is active
                        client.send_goal(original_goal)

                    #if waitCounter_ms % 50000 == 0 and not isSubGoalActive:
                    #    # Re-send the original goal every 5 second if no sub-goal is active
                    #    # print("Resending original goal...")
                    #    client.send_goal(original_goal)

                    if (gpt_output == 2 and waitCounter_ms % 15000 == 0) or (gpt_output != 2 and waitCounter_ms % 30000 == 0) :
                        rospy.loginfo("障害物回避サブゴール送信")
                        # 基準となる直線の方向を計算
                        if pose_index > 0:  # 現在のウェイポイントのインデックスが0でない場合
                            prev_pose = waypoints[pose_index - 1]
                            direction_x = pose[0][0] - prev_pose[0][0]
                            direction_y = pose[0][1] - prev_pose[0][1]
                            direction_length = math.sqrt(direction_x**2 + direction_y**2)
                            direction_x /= direction_length  # 正規化
                            direction_y /= direction_length  # 正規化

                            # 垂直方向のベクトルを計算
                            perp_x = -direction_y
                            perp_y = direction_x


                            # 左右の回避ゴールを計算
                            if float(pose[2][1]) > 0:  # 左が開いている場合
                                if offset_l <= float(pose[2][1]):  # フラグ内の距離で回避可能な場合のみ
                                    print("create sub-goal L")
                                    goal.target_pose.pose.position.x = pose_x + perp_x * offset_l
                                    goal.target_pose.pose.position.y = pose_y + perp_y * offset_l
                                    isSubGoalActive = True
                                else:
                                    goal.target_pose.pose.position.x = pose_x + perp_x * 1
                                    goal.target_pose.pose.position.y = pose_y + perp_y * 1
                                    isSubGoalActive = True
                                    print("Left offset too large, skipping avoidance.")
                            elif float(pose[2][0]) > 0:  # 右が開いている場合
                                if offset_r <= float(pose[2][0]):  # フラグ内の距離で回避可能な場合のみ
                                    print("create sub-goal R")
                                    goal.target_pose.pose.position.x = pose_x - perp_x * offset_r
                                    goal.target_pose.pose.position.y = pose_y - perp_y * offset_r
                                    isSubGoalActive = True
                                else:
                                    goal.target_pose.pose.position.x = pose_x - perp_x * 1
                                    goal.target_pose.pose.position.y = pose_y - perp_y * 1
                                    print("Right offset too large, skipping avoidance.")

                            # サブゴールを送信
                            if isSubGoalActive:
                                client.send_goal(goal)
                        else:
                            print("Cannot calculate sub-goal direction: no previous waypoint")


                    if waitCounter_ms % 70000 == 0:
                        rospy.loginfo("ネクストゴール")
                        # If no progress after 50 seconds, move to the next goal if conditions allow
                        if not pose[2][2] and not pose[2][3]:
                            if not pose[2][4]:
                                break

                    if isSubGoalActive == True:
                        # Check if sub-goal is reached within 0.1m tolerance
                        distance_to_subgoal = math.sqrt((pose_x - goal.target_pose.pose.position.x) ** 2 + (pose_y - goal.target_pose.pose.position.y) ** 2)
                        if distance_to_subgoal <= 0.55:
                            rospy.loginfo("サブゴール到達")
                            client.send_goal(original_goal)
                            isSubGoalActive = False
                            waitCounter_ms = 0

                    rospy.sleep(waitTime_ms / 1000)  # Sleep for waitTime_ms milliseconds
