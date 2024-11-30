#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time

class OdomImageSaver:
    def __init__(self):
        rospy.init_node('odom_image_saver', anonymous=True)

        # Initialize variables
        self.bridge = CvBridge()
        self.last_save_time = time.time()
        self.current_velocity = 0.0
        self.image_data = None

        # Directories and timing
        self.save_dir = os.getcwd()  # Current directory
        self.save_interval = 5  # seconds

        # Subscribers
        rospy.Subscriber('/ypspur_ros/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)

    def odom_callback(self, msg):
        # Calculate linear velocity magnitude
        vel = msg.twist.twist.linear
        self.current_velocity = (vel.x ** 2 + vel.y ** 2 + vel.z ** 2) ** 0.5
        print(self.current_velocity)

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV image
        try:
            self.image_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", str(e))

    def save_image_if_needed(self):
        # Check velocity and time
        if self.current_velocity <= 0.3 and time.time() - self.last_save_time >= self.save_interval:
            if self.image_data is not None:
                # Create a filename with a timestamp
                filename = os.path.join(self.save_dir, f'image_{int(time.time())}.jpeg')
                try:
                    # Save the image
                    cv2.imwrite(filename, self.image_data)
                    rospy.loginfo(f"Saved image: {filename}")
                    self.last_save_time = time.time()
                except Exception as e:
                    rospy.logerr(f"Failed to save image: {str(e)}")

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.save_image_if_needed()
            rate.sleep()

if __name__ == '__main__':
    try:
        saver = OdomImageSaver()
        saver.run()
    except rospy.ROSInterruptException:
        pass
