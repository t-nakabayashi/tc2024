#! /usr/bin/env python3
#coding=utf-8

import open3d as o3d
import pandas as pd
import numpy as np
from tqdm import tqdm
import rospy

# PCDファイルの読み込み（パスは適宜変更してください）
pcd_path = rospy.get_param("/pcd", '')
pcd = o3d.io.read_point_cloud(pcd_path)

# CSVファイルから走行軌跡を読み込む
csv_path = rospy.get_param("/waypoint", '')
waypoints = pd.read_csv(csv_path, usecols=['x', 'y', 'z'])

cleaned_pcd_path = rospy.get_param("/cleaned_pcd", '')
light_pcd_path = rospy.get_param("/light_pcd", '')  # 新しいPCDファイル保存用

original_points = np.asarray(pcd.points)

# まずは元データからランダムに10%の点を選択
num_points = original_points.shape[0]
sampled_indices = np.random.choice(num_points, size=num_points // 10, replace=False)
sampled_points = original_points[sampled_indices]

# ランダムサンプリングした点群を新しいPCDファイルとして保存
light_pcd = o3d.geometry.PointCloud()
light_pcd.points = o3d.utility.Vector3dVector(sampled_points)
o3d.io.write_point_cloud(light_pcd_path, light_pcd)

print(f'ランダムサンプリングされたPCDファイルが {light_pcd_path} に保存されました。')

# フィルタリング用の点群はサンプリングされた点群から処理
all_filtered_points = np.empty((0, 3))
for index, waypoint in tqdm(waypoints.iterrows(), total=waypoints.shape[0], desc="Filtering points"):
    x_min, x_max = waypoint['x'] - 7, waypoint['x'] + 7
    y_min, y_max = waypoint['y'] - 7, waypoint['y'] + 7
    z_min, z_max = waypoint['z'] - 0.5, waypoint['z'] + 0.3

    filtered_points = sampled_points[
        (x_min <= sampled_points[:, 0]) & (sampled_points[:, 0] <= x_max) &
        (y_min <= sampled_points[:, 1]) & (sampled_points[:, 1] <= y_max) &
        (z_min <= sampled_points[:, 2]) & (sampled_points[:, 2] <= z_max)
    ]

    all_filtered_points = np.vstack((all_filtered_points, filtered_points))

# 走行軌跡に沿った直線を生成し、半径1m以内の点を削除
for i in tqdm(range(len(waypoints) - 1), desc="Removing points near paths"):
    p1 = waypoints.iloc[i].to_numpy()
    p2 = waypoints.iloc[i + 1].to_numpy()

    # センサの高さ分だけウェイポイントの高さを下げる
    p1[2] -= 0.8
    p2[2] -= 0.8

    line_vec = p2 - p1
    line_vec_norm = line_vec / np.linalg.norm(line_vec)

    p1_reshape = p1.reshape(1, 3)
    distances = np.linalg.norm(np.cross(all_filtered_points - p1_reshape, line_vec_norm), axis=1)

    # 距離が2m以下の点を削除
    all_filtered_points = all_filtered_points[distances > 1]

# 重複を削除
all_filtered_points = np.unique(all_filtered_points, axis=0)

# フィルタリングされた点群を新しいPCDファイルとして保存
new_pcd = o3d.geometry.PointCloud()
new_pcd.points = o3d.utility.Vector3dVector(all_filtered_points)
o3d.io.write_point_cloud(cleaned_pcd_path, new_pcd)

print(f'フィルタリングされたPCDファイルが {cleaned_pcd_path} に保存されました。')
