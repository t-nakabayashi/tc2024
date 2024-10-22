#!/usr/bin/env python3

import rospy
import csv
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseActionGoal
import math

import tf
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

def quaternion_to_euler(q1, q2, q3, q4):
    """Convert Quaternion to Euler Angles

    quarternion: geometry_msgs/Quaternion
    euler: geometry_msgs/Vector3
    """
    e = tf.transformations.euler_from_quaternion((q1, q2, q3, q4))
    return Vector3(x=e[0], y=e[1], z=e[2])


def callback(data):
    pos = data.goal.target_pose.pose
    print ("{0},{1},0.0,0.0,0.0,{2},{3},".format(pos.position.x,pos.position.y,pos.orientation.z,pos.orientation.w))


rospy.init_node("waypoint_map_view")

pub = rospy.Publisher("waypoint", Marker, queue_size = 100)
rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, callback)

waypoint_name = rospy.get_param("~waypoint", '/home/nakaba/map/waypoint_tsukuba2023_lio11.csv')


rate = rospy.Rate(1)

while not rospy.is_shutdown():
    with open(waypoint_name, 'r') as f:
        counter = 0
        reader = csv.reader(f)
        header = next(reader)

        for row in reader:
            # Mark arrow
            marker_data = Marker()
            marker_data.header.frame_id = "map"
            marker_data.header.stamp = rospy.Time.now()

            marker_data.ns = "basic_shapes"
            marker_data.id = counter

            marker_data.action = Marker.ADD

            marker_data.pose.position.x = float(row[1])
            marker_data.pose.position.y = float(row[2])
            marker_data.pose.position.z = float(row[3])

            marker_data.pose.orientation.x=float(row[4])
            marker_data.pose.orientation.y=float(row[5])
            marker_data.pose.orientation.z=float(row[6])
            marker_data.pose.orientation.w=float(row[7])

            if int(row[10]) == 1:
                marker_data.color.r = 1.0
                marker_data.color.g = 0.0
                marker_data.color.b = 0.0
                marker_data.color.a = 1.0
                marker_data.scale.x = 2
                marker_data.scale.y = 0.1
                marker_data.scale.z = 0.1
            elif int(row[11]) == 1:
                marker_data.color.r = 1.0
                marker_data.color.g = 1.0
                marker_data.color.b = 0.0
                marker_data.color.a = 1.0
                marker_data.scale.x = 2
                marker_data.scale.y = 0.1
                marker_data.scale.z = 0.1
            elif int(row[9]) != 0 or int(row[8]) != 0 :
                marker_data.color.r = 0.0
                marker_data.color.g = 0.0
                marker_data.color.b = 1.0
                marker_data.color.a = 1.0
                marker_data.scale.x = 2
                marker_data.scale.y = 0.1
                marker_data.scale.z = 0.1
            else:
                marker_data.color.r = 0.0
                marker_data.color.g = 1.0
                marker_data.color.b = 0.0
                marker_data.color.a = 1.0
                marker_data.scale.x = 2
                marker_data.scale.y = 0.1
                marker_data.scale.z = 0.1

            marker_data.lifetime = rospy.Duration()

            marker_data.type = 0

            pub.publish(marker_data)
            counter +=1
            print(counter)


            # Mark num
            marker_data = Marker()
            marker_data.header.frame_id = "map"
            marker_data.header.stamp = rospy.Time.now()

            marker_data.ns = "basic_shapes"
            marker_data.id = counter

            marker_data.action = Marker.ADD

            marker_data.pose.position.x = float(row[1])
            marker_data.pose.position.y = float(row[2])
            marker_data.pose.position.z = float(row[3])

            marker_data.pose.orientation.x=float(row[4])
            marker_data.pose.orientation.y=float(row[5])
            marker_data.pose.orientation.z=float(row[6])
            marker_data.pose.orientation.w=float(row[7])

            marker_data.color.r = 0.0
            marker_data.color.g = 0.0
            marker_data.color.b = 0.0
            marker_data.color.a = 1.0
            marker_data.scale.x = 1
            marker_data.scale.y = 1
            marker_data.scale.z = 1

            marker_data.lifetime = rospy.Duration()

            marker_data.type = Marker.TEXT_VIEW_FACING
            marker_data.text = str(int(float(row[0])))

            pub.publish(marker_data)
            counter +=1

    rate.sleep()

rospy.spin()
