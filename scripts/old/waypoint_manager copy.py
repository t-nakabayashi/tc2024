#!/usr/bin/env python3

import rospy
import actionlib
import tf
from nav_msgs.msg import Odometry
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
import csv
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
import math
import random
from std_msgs.msg import Int32


waypoints = []
isGoalError = False
pose_x = 0
pose_y = 0
stopFlag = 0
boostFlag = 0

pub_vel = rospy.Publisher("/ypspur_ros/cmd_vel", Twist, queue_size = 10)
pub_init = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)


def quaternion_to_euler(q1, q2, q3, q4):
    """Convert Quaternion to Euler Angles0

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((q1, q2, q3, q4))
    return Vector3(x=e[0], y=e[1], z=e[2])

def goal_pose(pose):
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
    global isGoalError
    isGoalError = False
    for status in data.status_list:
        if status.status >= 4:
            isGoalError = True
        else:
            pass

def velCallBack(data):
    # 速度を最パブリッシュする部分を作成
    if boostFlag == 1:
        data.linear.x *= 1.0
    elif stopFlag == 1:
        print("stop_flag_on")
        data.linear.x = 0
        data.angular.x = 0
        data.angular.y = 0
        data.angular.z = 0
    else:
        data.linear.x *= 1.0

    pub_vel.publish(data)



def mclposeCallBack(data):
    global pose_x,pose_y
    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y


if __name__ == '__main__':
    rospy.init_node('patrol')
    #listener = tf.TransformListener()
    #global stopFlag
    #global boostFlag

    pub_recog = rospy.Publisher('recog_flag', Int32, queue_size=1)


    waitCounter_ms = 0
    waitTime_ms = 50
    start_num = rospy.get_param("~start_num", 0)
    print (start_num)
    waypoint_name = rospy.get_param("~waypoint", '/home/nakaba/map/waypoint_tsukuba2023.csv')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    #listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(3.0))

    rospy.Subscriber('/move_base/status',GoalStatus, goalstatusCallBack)
    rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped, mclposeCallBack)
    rospy.Subscriber('/ypspur_ros/cmd_vel_old',Twist, velCallBack)

    with open(waypoint_name, 'r') as f:
        counter = 0
        reader = csv.reader(f)
        header = next(reader)

        for row in reader:
            if counter < start_num:
                counter+=1
            else :
                waypoints.append([
                                (float(row[1]),float(row[2]),0.0),
                                (0.0,0.0,float(row[6]),float(row[7])),
                                (int(row[8]),int(row[9]),int(row[10]),int(row[11]),int(row[12])),
                                (int(row[0]))
                                #0 right_open_range	1 left_open_range  	2 line_is_stop  	3 signal_is_stop	 4 isnot_skip
                                 ])

    while not rospy.is_shutdown():  # rospy.is_shutdown() を使う

        for pose in waypoints:
            boostFlag = 0
            goal = goal_pose(pose)
            client.send_goal(goal)
            status_string = "num:" + str(pose[3])
            pose_string = (" right_open "," left_open "," line_stop "," signal_stop "," not_skip ")
            for i,p in enumerate(pose[2]):
                if p != 0:
                    status_string += pose_string[i]
                    if i <= 1:
                        status_string += str(p) + "mm,"
            print(status_string)
            #print ("num:" + str(pose[3]) + " line_is_stop:" + str(pose[2][2]) + " signal_is_stop:" + str(pose[2][3]))

            while True:
                if int(pose[3]) < 1:
                    break
                # mcl_poseを使用するように変更
                print(math.sqrt((pose_x-goal.target_pose.pose.position.x)**2 + (pose_y-goal.target_pose.pose.position.y)**2 ))
                if(math.sqrt((pose_x-goal.target_pose.pose.position.x)**2 + (pose_y-goal.target_pose.pose.position.y)**2 ) <= 1.3):
                    if pose[2][2] == False and pose[2][3] == False:
                        # isstopが両方Fである場合はすぐに次に行く
                        print("next")
                        waitCounter_ms = 0
                        break
                    elif pose[2][2] == True:
                        if(math.sqrt((pose_x-goal.target_pose.pose.position.x)**2 + (pose_y-goal.target_pose.pose.position.y)**2 ) <= 0.5):
                            stopFlag = 1

                            print ("wait!!")
                            input_data = input()
                            stopFlag = 0
                            waitCounter_ms = 0
                            break
                    elif pose[2][3] == True:
                        if(math.sqrt((pose_x-goal.target_pose.pose.position.x)**2 + (pose_y-goal.target_pose.pose.position.y)**2 ) <= 0.5):
                            print("signal!!!")#! /usr/bin/env python3
# coding=utf-8

import rospy
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from std_msgs.msg import Int32

class ROSParamGUI:
    def __init__(self, master):
        self.master = master
        master.title("ROS Parameter Controller")

        # パラメータとそのラベル
        self.params = {
            '/right_is_open': {'label': 'Right Is Open', 'default': 0},
            '/left_is_open': {'label': 'Left Is Open', 'default': 0},
            '/line_is_stop': {'label': 'Line Is Stop', 'default': 0},
            '/signal_is_stop': {'label': 'Signal Is Stop', 'default': 0},
            '/isnot_skipnum': {'label': 'Is Not Skip Num', 'default': 0}
        }

        self.entries = {}
        self.publishers = {}

        row = 0
        for param, info in self.params.items():
            label = ttk.Label(master, text=info['label'])
            label.grid(row=row, column=0, padx=10, pady=5, sticky=tk.W)

            var = tk.IntVar()
            var.set(info['default'])
            self.entries[param] = var

            checkbox = ttk.Checkbutton(master, variable=var)
            checkbox.grid(row=row, column=1, padx=10, pady=5)
            row += 1

            # 各パラメータ用のトピックを作成
            topic_name = param.replace('/', '')  # '/'を取り除いたトピック名
            self.publishers[param] = rospy.Publisher(topic_name, Int32, queue_size=10)

        self.update_button = ttk.Button(master, text="Update Parameters", command=self.update_parameters)
        self.update_button.grid(row=row, column=0, columnspan=2, pady=10)

        # 0.5秒周期でパブリッシュを行うタイマー
        rospy.Timer(rospy.Duration(0.5), self.publish_params)

    def update_parameters(self):
        try:
            for param, var in self.entries.items():
                value = var.get()
                # 値が更新された際にトピックに値をpublish
                self.publishers[param].publish(value)
            messagebox.showinfo("Success", "Parameters updated and published successfully!")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to update parameters:\n{e}")

    def publish_params(self, event):
        # 0.5秒周期で全てのパラメータをパブリッシュ
        for param, var in self.entries.items():
            value = var.get()
            self.publishers[param].publish(value)

def main():
    rospy.init_node('ros_param_gui', anonymous=True)

    root = tk.Tk()
    gui = ROSParamGUI(root)
    root.mainloop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

                            stopFlag = 1
                            recog = 0
                            while(recog != 1):
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

                            stopFlag = 0
                            waitCounter_ms = 0
                            break
                else:
                    pub_recog.publish(0)
                    waitCounter_ms += waitTime_ms
                    if isGoalError == True:
                        # ステータスがエラーであればゴールを再送する
                        client.send_goal(goal)

                    if waitCounter_ms%5000 == 0:
                        # １０秒おきにゴールを再送する

                        if waitCounter_ms%10000 == 0:
                            # 10秒おきにゴールを移動する
                            theta = quaternion_to_euler(pose[1][0],pose[1][1],pose[1][2],pose[1][3])

                            seed = random.random()
                            if int(pose[2][1]) == 1:
                                #　左が空いている場合
                                print("move waypoint L")
                                goal.target_pose.pose.position.x = pose[0][0] + seed*float(pose[2][1])/2000.0 * math.cos(theta.z+1.57078)
                                goal.target_pose.pose.position.y = pose[0][1] + seed*float(pose[2][1])/2000.0 * math.sin(theta.z+1.57078)

                            elif  int(pose[2][0]) == 1:
                                #　右が空いている場合
                                print("move waypoint R")
                                goal.target_pose.pose.position.x = pose[0][0] + seed*float(pose[2][0])/2000.0 * math.cos(theta.z-1.57078)
                                goal.target_pose.pose.position.y = pose[0][1] + seed*float(pose[2][0])/2000.0 * math.sin(theta.z-1.57078)

                        # ゴールを再送する
                        client.send_goal(goal)

                        if waitCounter_ms%300000 == 0:
                            # 50秒ゴールに到達しなかったら次のゴールを送る
                            if pose[2][2] == False and pose[2][3] == False:
                                # isstopが両方共Fであるときだけ次のゴールに行く
                                if pose[2][4] == False:
                                    # isnot skipがFであるときだけ次のゴールに行く
                                    break

                    rospy.sleep(waitTime_ms/1000)
