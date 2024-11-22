#!/usr/bin/env python3

import rospy
import csv
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseActionGoal
import math
import tf
from geometry_msgs.msg import Quaternion, Vector3

def quaternion_to_euler(q1, q2, q3, q4):
    """Convert Quaternion to Euler Angles"""
    e = tf.transformations.euler_from_quaternion((q1, q2, q3, q4))
    return Vector3(x=e[0], y=e[1], z=e[2])

class WaypointMapView:
    def __init__(self):
        rospy.init_node("waypoint_map_view")
        self.pub = rospy.Publisher("waypoint", Marker, queue_size=100)
        self.goal_marker_pub = rospy.Publisher("goal_marker", Marker, queue_size=10)
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.goal_callback)

        self.waypoint_name = rospy.get_param("~waypoint", '/home/nakaba/map/waypoint_tsukuba2023_lio11.csv')
        self.rate = rospy.Rate(1)

        self.current_goal_marker_id = 0  # Unique ID for goal marker

    def goal_callback(self, data):
        """Callback for /move_base/goal"""
        pos = data.goal.target_pose.pose.position

        # Create a red circular marker for the goal
        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = rospy.Time.now()

        marker_data.ns = "goal_marker"
        marker_data.id = self.current_goal_marker_id  # Unique ID for this marker
        marker_data.type = Marker.SPHERE
        marker_data.action = Marker.ADD

        marker_data.pose.position.x = pos.x
        marker_data.pose.position.y = pos.y
        marker_data.pose.position.z = 0.2  # Slightly above ground level for visibility

        marker_data.scale.x = 0.5  # Diameter of the sphere
        marker_data.scale.y = 0.5
        marker_data.scale.z = 0.5

        marker_data.color.r = 1.0
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0  # Fully opaque

        marker_data.lifetime = rospy.Duration()  # Marker persists indefinitely

        # Publish the goal marker
        self.goal_marker_pub.publish(marker_data)
        rospy.loginfo(f"Published goal marker at x: {pos.x}, y: {pos.y}")

    def run(self):
        """Main loop to publish waypoints"""
        while not rospy.is_shutdown():
            with open(self.waypoint_name, 'r') as f:
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

                    marker_data.pose.orientation.x = float(row[4])
                    marker_data.pose.orientation.y = float(row[5])
                    marker_data.pose.orientation.z = float(row[6])
                    marker_data.pose.orientation.w = float(row[7])

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
                    elif float(row[9]) > 0:
                        if float(row[9]) < 1:
                            marker_data.color.r = 0.0
                            marker_data.color.g = 0.0
                            marker_data.color.b = 1.0
                            marker_data.color.a = 1.0
                            marker_data.scale.x = 1
                            marker_data.scale.y = 0.1
                            marker_data.scale.z = 0.1
                        else:
                            marker_data.color.r = 0.0
                            marker_data.color.g = 0.0
                            marker_data.color.b = 1.0
                            marker_data.color.a = 1.0
                            marker_data.scale.x = 2
                            marker_data.scale.y = 0.1
                            marker_data.scale.z = 0.1
                    elif float(row[8]) > 0:
                        if float(row[8]) < 1:
                            marker_data.color.r = 1.0
                            marker_data.color.g = 1.0
                            marker_data.color.b = 1.0
                            marker_data.color.a = 1.0
                            marker_data.scale.x = 1
                            marker_data.scale.y = 0.1
                            marker_data.scale.z = 0.1
                        else:
                            marker_data.color.r = 1.0
                            marker_data.color.g = 1.0
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

                    self.pub.publish(marker_data)
                    counter += 1
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

                    marker_data.pose.orientation.x = float(row[4])
                    marker_data.pose.orientation.y = float(row[5])
                    marker_data.pose.orientation.z = float(row[6])
                    marker_data.pose.orientation.w = float(row[7])

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

                    self.pub.publish(marker_data)
                    counter += 1

            self.rate.sleep()


if __name__ == "__main__":
    node = WaypointMapView()
    node.run()
