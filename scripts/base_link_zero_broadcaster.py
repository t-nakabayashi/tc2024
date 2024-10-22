#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import LaserScan

class BaseLinkZero:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        self.broadcaster = tf2_ros.TransformBroadcaster()

    def get_transform(self):
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            return trans
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("TF lookup failed")
            return None

    def broadcast_zero_z(self):
        trans = self.get_transform()
        if trans:
            trans.child_frame_id = "base_link_zero"
            trans.transform.translation.z = 1
            self.broadcaster.sendTransform(trans)



def callback(msg):
    # tfのフレームIDを変更
    #msg.header.frame_id = "base_link_zero"
    # 変更されたメッセージを再公開
    #pub.publish(msg)

    # 現在のROS時刻を取得
    #current_time = rospy.Time.now()
    msg.header.stamp = rospy.Time.now()

    # tfのフレームIDを変更
    msg.header.frame_id = "base_link_zero"

    # ヘッダーのタイムスタンプを現在のROS時刻に設定
    #msg.header.stamp = current_time

    # 変更されたメッセージを再公開
    pub.publish(msg)



if __name__ == '__main__':
    rospy.init_node('base_link_zero_broadcaster')

    base_link_zero = BaseLinkZero()
    # /scan トピックを購読
    sub = rospy.Subscriber('/scan_livox_front_low_move', LaserScan, callback)
    # /scan_modified としてメッセージを公開
    pub = rospy.Publisher('/scan_livox_front_low_move_zero', LaserScan, queue_size=10)


    rate = rospy.Rate(20)  # 10Hzの更新頻度で実行
    while not rospy.is_shutdown():
        base_link_zero.broadcast_zero_z()
        rate.sleep()
