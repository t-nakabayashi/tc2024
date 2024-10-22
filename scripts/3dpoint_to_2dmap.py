#! /usr/bin/env python3
#coding=utf-8

import rospy
import open3d as o3d
import numpy as np
import cv2
import yaml
import os  # パス操作のためにosをインポート
from tqdm import tqdm  # 進捗表示のためにtqdmをインポート

# ポイントクラウドデータを2Dマップに変換する関数
def pointcloud_to_map(point_cloud, resolution, map_width, map_height, noise_threshold):
    # Z軸を無視してXY平面に投影
    points = np.array(point_cloud.points)
    points_2d = points[:, :2]  # XY平面に投影（Z軸を無視）

    # マップサイズを定義（初期値は白）
    map_image = np.ones((map_height, map_width), dtype=np.uint8) * 255

    # グリッドごとのポイント数をカウントするためのマトリックス
    point_count = np.zeros((map_height, map_width), dtype=np.int32)

    # 原点を画像の中心に移動するためのオフセット計算
    map_center_x = map_width // 2
    map_center_y = map_height // 2

    # 各ポイントをグリッドに変換し、カウント
    for point in tqdm(points_2d, desc="Processing points", unit="points"):
        # 解像度を使用して座標をピクセル単位に変換
        x = int(point[0] / resolution) + map_center_x
        y = int(point[1] / resolution) + map_center_y

        # Y座標を反転させる
        y = map_height - y  # 上下反転

        # 座標がマップの範囲内か確認して、ポイント数をカウント
        if 0 <= x < map_width and 0 <= y < map_height:
            point_count[y, x] += 1

    # ポイント数に基づいてマップを作成
    for y in tqdm(range(map_height), desc="Generating map", unit="rows"):
        for x in range(map_width):
            if point_count[y, x] > noise_threshold:
                map_image[y, x] = 0  # 障害物として黒く塗る
            else:
                map_image[y, x] = 255  # ノイズとして白にする

    return map_image

# YAMLファイルを作成する関数
def save_yaml(file_path, resolution, origin, map_file):
    # ファイル名だけを取得（パスを削除）
    map_file_name = os.path.basename(map_file)

    data = {
        'image': map_file_name,  # ファイル名のみを指定
        'resolution': resolution,
        'origin': origin,
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }
    
    with open(file_path, 'w') as yaml_file:
        yaml.dump(data, yaml_file, default_flow_style=False)

def main():
    # ROSノードを初期化
    rospy.init_node('pointcloud_to_map')

    # ROSパラメータから値を取得
    pointcloud_file = rospy.get_param('/cleaned_pcd_file', '/path/to/pointcloud.pcd')
    map_file = rospy.get_param('/map_file', '/path/to/output_map.png')
    yaml_file = rospy.get_param('/yaml_file', '/path/to/output_map.yaml')
    resolution = rospy.get_param('/resolution', 0.05)  # 1セルあたりのメートル数
    map_width = rospy.get_param('/map_width', 1024)  # マップの幅（ピクセル）
    map_height = rospy.get_param('/map_height', 1024)  # マップの高さ（ピクセル）
    noise_threshold = rospy.get_param('/noise_threshold', 1)  # ノイズ除去のしきい値

    # ポイントクラウドを読み込む
    cloud = o3d.io.read_point_cloud(pointcloud_file)

    # 2Dマップに変換（ノイズ除去も含む）
    map_image = pointcloud_to_map(cloud, resolution, map_width, map_height, noise_threshold)

    # PNGファイルとして保存
    cv2.imwrite(map_file, map_image)

    # 原点を画像の中央に設定し、YAML用のoriginを計算
    origin_x = -map_width // 2 * resolution
    origin_y = -map_height // 2 * resolution
    origin = [origin_x, origin_y, 0.0]  # 原点を画像の中央に合わせる

    # YAMLファイルを生成して保存
    save_yaml(yaml_file, resolution, origin, map_file)

    rospy.loginfo(f"2D map and YAML file saved successfully with origin: {origin}")

if __name__ == '__main__':
    main()
