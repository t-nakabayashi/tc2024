#!/usr/bin/python3

import rospy
from std_msgs.msg import Bool, Int32
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import torch
import argparse
import time

# "/usb_cam/image_raw"という名前のROSトピックを画像として受信
def receive_image():
    image_msg = rospy.wait_for_message('/usb_cam/image_raw', Image)
    image = np.frombuffer(image_msg.data, dtype=np.uint8).reshape(image_msg.height, image_msg.width, 3)
    return image

# YOLOv5による物体検出を実行
def detect_object(image):
    y = model(image)
    detections = y.pandas().xyxy[0]  

    return detections

# 検出結果を1つに絞る
def align_detections(detections):

    if len(detections) > 0:
        conf_max_id = 0
        conf_max = 0
        for i in range(len(detections)):
            if detections["confidence"][i] > conf_max:
                conf_max_id = i
        aligned = detections["class"][conf_max_id]
    else:
        aligned = 99

    return aligned

# 何か検出された場合は、"det_status"にはそのクラスIDを代入し発行
def publish_det_status(detections):
    det_status = rospy.Publisher('det_status', Int32, queue_size=1)
    if len(detections) > 0:
        # 確信度が最も高い検出候補を採用
        conf_max_id = 0
        conf_max = 0
        for i in range(len(detections)):
            if detections["confidence"][i] > conf_max:
                conf_max_id = i
        
        # 確信度が閾値を超えなかった場合は未検出扱い
        if detections['class'][conf_max_id] < det_thresh:
            det_status.publish(99)
        # 確信度が閾値を超えたときに検出クラス情報を発行
        else:    
            det_status.publish(detections['class'][conf_max_id])
    else:
        det_status.publish(99)

# 画像上に物体検出結果を重畳表示する
def overlay_detections(image, detections, sig_recog):
    
    # 信号横断の判定結果を描画する
    if sig_recog == 2: # NOGO
        sig_recog_color = (255, 0, 0)
    else: # GO
        sig_recog_color = (0, 255, 0)

    cv2.rectangle(image, (0, 0), ((int)(image.shape[1]), (int)(image.shape[0])), sig_recog_color, 5)

    # 当該フレームで検出された信号の矩形を描画する
    for i in range(len(detections)):

        # 閾値以下の候補は描画しない
        if detections['confidence'][i] < det_thresh:
            continue

        # 信号の検出矩形を描画する
        if detections['name'][i] == "red":
            det_color = (255, 0, 0)
        else:
            det_color = (0, 255, 0)

        cv2.rectangle(image, ((int)(detections['xmax'][i]), (int)(detections['ymax'][i])), ((int)(detections['xmin'][i]), (int)(detections['ymin'][i])), det_color, 2)
        
        # クラス名を描画する
        #det_str = detections['name'][i] + "," + '{:.2f}'.format(detections['confidence'][i])
        det_str = '{:.2f}'.format(detections['confidence'][i]) # 検出結果
        det_str += ", " + str(sig_recog) # 判定結果
        cv2.putText(image, det_str, ((int)(detections['xmax'][i]), (int)(detections['ymax'][i])), cv2.FONT_HERSHEY_SIMPLEX, 1, det_color, 5)

# 画像を"/sig_det_imgs"という名前のトピックで出力する
def publish_det_imgs(image):
    img_pub = rospy.Publisher('sig_det_imgs', Image, queue_size=10)

    bridge = CvBridge()
    proc_msg = bridge.cv2_to_imgmsg(image, encoding="rgb8")
    img_pub.publish(proc_msg)

# stop/startの判定フラグを立てる
def judge_sig(det_list, cnt, isJudged):

    # 3回連続で1(Green)が出たときは"1"(start)とする
    # すでにstart判定がされたときも判定を保持する
    if sum(det_list) == cnt or isJudged == True: 
        sig_recog = 1
        #isJudged = True
    else: # デフォルトは"2"(stop)とする
        sig_recog = 2

    return sig_recog, isJudged

# sig_recogのトピックを発行
def publish_sig_recog(sig_recog):
    sig_pub = rospy.Publisher('sig_recog', Int32, queue_size=1)
    sig_pub.publish(sig_recog)

# "recog_flag"という名前のROSトピックを受信するまで処理を待機
#def wait_for_recog_flag():
#    while True:
#        recog_flag = rospy.wait_for_message('recog_flag', Int32, timeout=None)
#        if recog_flag.data == 1:
#            break

def monitoring_recog_flag():
    flag = rospy.wait_for_message('recog_flag', Int32)
    return flag

def wait_for_recog_flag():
    while True:
        recog_flag = monitoring_recog_flag()
        print ("recog_flag: ", recog_flag)
        if recog_flag.data == 1:
            break
        else:
            image = receive_image()
            publish_det_imgs(image)

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--conf', default=0.80)
    parser.add_argument('-j', '--judge', default=3)
    args = parser.parse_args()
    det_thresh = float(args.conf)
    judge_cnt = int(args.judge)

    # ROSを初期化する
    rospy.init_node('object_detection')

    # YOLOv5で学習されたモデルを読み込む
    model = torch.hub.load('ultralytics/yolov5', 'custom', path='best.pt')
    
    # モデルの検出閾値を設定
    model.conf = det_thresh

    # 本ループ処理
    while True:
        print ("wait for recog_flag...")

        # "recog_flag"という名前のROSトピックを受信するまで処理を待機
        wait_for_recog_flag()

        print ("detection start")

        pred_sigs = []
        isJudged = False
        
        # 検出ループ処理
        # recog_flagが1のとき無限ループ
        while (rospy.wait_for_message('recog_flag', Int32, timeout=None)).data == 1:

            start = time.time()

            # "/usb_cam/image_raw"という名前のROSトピックを画像として受信
            image = receive_image()

            # YOLOv5による物体検出を実行
            detections = detect_object(image)
            print (detections)
                  
            # 検出結果をstart/stop判定用の配列に格納
            pred_sigs.append(align_detections(detections))
            print ("pred_sigs: ", pred_sigs)
            
            # start/stopを判定
            sig_recog, isJudged = judge_sig(pred_sigs, judge_cnt, isJudged)
            print ("sig_recog: ", sig_recog)

            # 要素が指定した判定回数を超えたら古いものから削除
            if len(pred_sigs) >= 3:
                pred_sigs.pop(0)
            
            # 判定結果をpublish
            publish_sig_recog(sig_recog)

            # (デバッグ用) 生の検出結果をpublish
            #publish_det_status(detections)
            
            # (デバッグ用) 画像上に物体検出&start/stopの判定結果を重畳表示する
            overlay_detections(image, detections, sig_recog)

            # (デバッグ用)  画像を"/sig_det_imgs"という名前のトピックで出力する
            publish_det_imgs(image)

            end = time.time()

            proc_time = (end - start) * 1000

            print ("Process time: {:.2f} [ms]".format(proc_time))
            
        print ("detection end")