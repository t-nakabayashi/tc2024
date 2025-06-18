#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud2, PointField, Imu


# パブリッシャの初期化
pub = None
pub2 = None
pub2 = rospy.Publisher('/mid360/livox/imu_new', Imu, queue_size=10)


# LivoxからのPointCloudデータをそのままパブリッシュするコールバック
def livox_callback(msg):
    msg.header.frame_id = "mid360_frame_new"
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)

# VelodyneからのPointCloudデータをそのままパブリッシュするコールバック
def velodyne_callback(msg):
    pass
    #pub.publish(msg)

def callback2(msg):
    msg.header.frame_id = "mid360_frame_new"
    msg.header.stamp = rospy.Time.now()
    pub2.publish(msg)

# ROSノードの初期化
def listener():
    global pub
    rospy.init_node('pointcloud_forwarder', anonymous=True)

    # パブリッシャの作成
    pub = rospy.Publisher('/merged_points', PointCloud2, queue_size=10)

    # LivoxとVelodyneのPointCloud2トピックを購読
    rospy.Subscriber("/mid360/livox/lidar", PointCloud2, livox_callback)
    rospy.Subscriber("/velodyne_points", PointCloud2, velodyne_callback)
    rospy.Subscriber("/mid360/livox/imu", Imu, callback2)

    # ROSのスピン（購読を維持）
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
