#! /usr/bin/env python3
#coding=utf-8

import rospy
from std_srvs.srv import Empty

def clear_costmaps():
    try:
        # サービスクライアントを作成
        rospy.wait_for_service('/move_base/clear_costmaps')
        clear_costmaps_service = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

        # ROSパラメータからクリアの頻度を取得（デフォルトは1秒）
        clear_interval_sec = rospy.get_param('~clear_interval_sec', 1)

        # 指定した頻度でサービスを呼び出す
        while not rospy.is_shutdown():
            clear_costmaps_service()
            rospy.sleep(clear_interval_sec)
            print("clear done")

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    try:
        # ROSノードの初期化
        rospy.init_node('clear_costmaps_client')
        clear_costmaps()
    except rospy.ROSInterruptException:
        pass
