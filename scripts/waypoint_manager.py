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
import random
from std_msgs.msg import Int32

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
        data: GoalStatus message.
    """
    global isGoalError
    isGoalError = False
    for status in data.status_list:
        if status.status >= 4:  # Status >= 4 indicates failure (e.g., ABORTED, REJECTED)
            isGoalError = True

def velCallBack(data):
    """
    Callback function to adjust the robot's velocity based on flags.

    Args:
        data: Twist message with velocity information.
    """
    if boostFlag == 1:
        # Boost mode is on, adjust the speed
        data.linear.x *= 1.0  # Currently not actually boosting, could change multiplier
    elif stopFlag == 1:
        # Stop the robot completely
        print("stop_flag_on")
        data.linear.x = 0
        data.angular.x = 0
        data.angular.y = 0
        data.angular.z = 0
    else:
        data.linear.x *= 1.0  # No change to speed

    # Publish the adjusted velocity
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
    rospy.Subscriber('/move_base/status', GoalStatus, goalstatusCallBack)
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, mclposeCallBack)
    rospy.Subscriber('/ypspur_ros/cmd_vel_old', Twist, velCallBack)

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
                    (int(row[8]), int(row[9]), int(row[10]), int(row[11]), int(row[12])),  # Flags for conditions
                    (int(row[0]))  # Waypoint number
                ])

    # Main loop for patrolling through waypoints
    while not rospy.is_shutdown():
        for pose in waypoints:
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

            # Wait until the goal is reached or some condition is met
            while not rospy.is_shutdown():
                if int(pose[3]) < 1:
                    break
                # Calculate the distance to the current goal
                distance_to_goal = math.sqrt((pose_x - goal.target_pose.pose.position.x) ** 2 + (pose_y - goal.target_pose.pose.position.y) ** 2)
                
                if distance_to_goal <= 0.5:  # Close enough to the goal
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
                            input_data = input()  # Wait for user input to continue
                            stopFlag = 0
                            waitCounter_ms = 0
                            break
                    elif pose[2][3]:
                        # Signal stop flag is True
                        if distance_to_goal <= 0.6:
                            print("signal!!!")
                            stopFlag = 1
                            recog = 0
                            while recog != 1:
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
                    # Update waiting status and re-send goal if necessary
                    pub_recog.publish(0)

                    waitCounter_ms += waitTime_ms

                    if isGoalError and not isSubGoalActive:
                        # Re-send original goal if there was an error and no sub-goal is active
                        client.send_goal(original_goal)

                    if waitCounter_ms % 5000 == 0 and not isSubGoalActive:
                        # Re-send the original goal every 1 second if no sub-goal is active
                        #print("Resending original goal...")
                        client.send_goal(original_goal)

                    if waitCounter_ms % 20000 == 0:
                        # After 20 seconds, if no progress, create a sub-goal 1m to the open side
                        theta = quaternion_to_euler(pose[1][0], pose[1][1], pose[1][2], pose[1][3])
                        if int(pose[2][1]) == 1:
                            # If left is open, create a sub-goal 1m to the left
                            print("create sub-goal L")
                            goal.target_pose.pose.position.x = pose_x + math.cos(theta.z + 1.57078) * 1.0
                            goal.target_pose.pose.position.y = pose_y + math.sin(theta.z + 1.57078) * 1.0
                            isSubGoalActive = True
                        elif int(pose[2][0]) == 1:
                            # If right is open, create a sub-goal 1m to the right
                            print("create sub-goal R")
                            goal.target_pose.pose.position.x = pose_x + math.cos(theta.z - 1.57078) * 1.0
                            goal.target_pose.pose.position.y = pose_y + math.sin(theta.z - 1.57078) * 1.0
                            isSubGoalActive = True

                        # Send the sub-goal
                        client.send_goal(goal)

                    if waitCounter_ms % 60000 == 0:
                        # If no progress after 50 seconds, move to the next goal if conditions allow
                        if not pose[2][2] and not pose[2][3]:
                            if not pose[2][4]:
                                break
                    else:
                        # Check if sub-goal is reached within 0.3m tolerance
                        distance_to_subgoal = math.sqrt((pose_x - goal.target_pose.pose.position.x) ** 2 + (pose_y - goal.target_pose.pose.position.y) ** 2)
                        if distance_to_subgoal <= 0.3:
                            print("Sub-goal reached, resending original goal")
                            client.send_goal(original_goal)
                            isSubGoalActive = False
                            waitCounter_ms = 0

                    rospy.sleep(waitTime_ms / 1000)  # Sleep for waitTime_ms milliseconds
