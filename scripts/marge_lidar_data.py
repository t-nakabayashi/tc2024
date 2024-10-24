#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2

# パブリッシャの初期化
pub = None

# LivoxからのPointCloudデータをそのままパブリッシュするコールバック
def livox_callback(msg):
    pub.publish(msg)

# VelodyneからのPointCloudデータをそのままパブリッシュするコールバック
def velodyne_callback(msg):
    pub.publish(msg)

# ROSノードの初期化
def listener():
    global pub
    rospy.init_node('pointcloud_forwarder', anonymous=True)

    # パブリッシャの作成
    pub = rospy.Publisher('/merged_points', PointCloud2, queue_size=10)

    # LivoxとVelodyneのPointCloud2トピックを購読
    rospy.Subscriber("/mid360/livox/lidar", PointCloud2, livox_callback)
    rospy.Subscriber("/velodyne_points", PointCloud2, velodyne_callback)

    # ROSのスピン（購読を維持）
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
