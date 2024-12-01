#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import base64
import time
from openai import OpenAI
import os
from os.path import join, dirname
from dotenv import load_dotenv
from cv_bridge import CvBridge
import cv2


# .envファイルからAPIキーを読み込む
dotenv_path = join(dirname(__file__), '.env')
load_dotenv(dotenv_path)
API_KEY = os.environ.get("API_KEY")

if not API_KEY:
    rospy.logerr("APIキーが見つかりません。.envファイルに設定してください。")
    exit(1)

# OpenAIクライアントの初期化
client = OpenAI(api_key=API_KEY)

class ObstacleDetector:
    def __init__(self):
        rospy.init_node('obstacle_detector', anonymous=True)
        
        # Publisher
        self.pub = rospy.Publisher('/gpt_output', Int32, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/ypspur_ros/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        
        # 状態変数
        self.current_speed = 0.0
        self.stationary_time = 0.0
        self.last_gpt_output_time = 0.0
        self.gpt_output = 99
        
        # 設定
        self.stationary_threshold = 0.3
        self.stationary_duration = 1.0
        self.gpt_request_interval = 10.0
        self.gpt_output_expiry = 1.0  # 1秒ごとに出力更新
        
        # 画像バッファ
        self.last_image = None

    def odom_callback(self, msg):
        # ロボットのX方向速度を取得
        self.current_speed = abs(msg.twist.twist.linear.x)
        #print(self.current_speed)
        if self.current_speed < self.stationary_threshold:
            self.stationary_time += 1.0 / 10.0  # 約10Hzのコールバックを仮定
        else:
            self.stationary_time = 0.0

    def image_callback(self, msg):
        self.last_image = msg  # 最新の画像を保存

    def encode_image(self, image_msg):
        """
        ROSの画像メッセージをJPEG形式に変換し、Base64でエンコードする。
        """
        try:
            # CvBridgeを使ってROS画像メッセージをOpenCV形式に変換
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

            # JPEG形式にエンコード
            _, buffer = cv2.imencode('.jpg', cv_image)
            
            # Base64エンコード
            encoded_image = base64.b64encode(buffer).decode('utf-8')
            
            rospy.loginfo("画像エンコード成功")
            return encoded_image
        except Exception as e:
            rospy.logerr(f"画像エンコードエラー: {e}")
            return None

    # GPT-4Oに画像とテキストプロンプトを送信する関数
    def send_image_and_prompt(self):
        """
        画像（Base64エンコード）とテキストプロンプトをGPT-4Oに送信する関数。
        """

        self.prompt_text = "あなたはロボットオペレータです。ロボットが前方１ｍ以内に障害物を検出して止まっており、その処理を考えてください\
        ロボットのセンサは必ずしも１００％の信頼性があるわけではないため、誤検知の可能性もあります。\
        ロボットの前方に取り付けられたカメラの映像をもとにロボットの次の動作を判断し、\
        次のどのような行動を起こすべきかを１行目に数字だけで回答してください。\
        誤検知の可能性が高く、特に障害物がないので前に進んでも良さそうな場合は0,\
        障害物が前方にありそう、かつ、それが人やロボット等の移動障害物の場合にはしばらく待機して相手の移動を待つ戦略を取り、数値としては1、\
        カラーコーン等の静止障害物の場合は、回避する必要があるため2を出力し、改行して理由を出力してください\
        その際、わかるようであれば右か左かどちら方向に回避すればよりスムーズに回避できるかも教えてください"  

        if not self.last_image:
            rospy.logwarn("画像がありません。")
            return
        
        # 画像をエンコード
        encoded_image = self.encode_image(self.last_image)
        if not encoded_image:
            rospy.logerr("画像のエンコードに失敗しました。")
            return

        # リクエストの作成
        try:
            # APIリクエストを送信
            response = client.chat.completions.create(
                model="gpt-4o-mini",  # モデルの指定
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": self.prompt_text},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{encoded_image}"
                                },
                            },
                        ],
                    }
                ],
            )

            # GPTの出力を解析
            response_message = response.choices[0].message.content
            response_text = response_message.encode("utf-8").decode("utf-8") if response_message else "No content returned"
              
            # 最初の数字をパース
            gpt_output = int(response_text.split()[0])
            self.gpt_output = gpt_output  # 結果を更新
            self.last_gpt_output_time = rospy.get_time()  # 成功時にタイムスタンプを更新
            
            rospy.loginfo(f"GPTの応答: {response_text}")
        except Exception as e:
            rospy.logerr(f"GPTリクエストエラー: {e}")
            self.gpt_output = 99

        except Exception as e:
            print(f"リクエスト中にエラーが発生しました: {e}")

    def publish_gpt_output(self):
        """
        1秒ごとにGPTの出力をROSトピックにパブリッシュする。
        """
        current_time = rospy.get_time()
        
        # ロボットが動いている場合、デフォルト値を送信
        if self.current_speed > self.stationary_threshold:
            self.gpt_output = 99
        
        # 1秒ごとに出力を更新
        if current_time - self.last_gpt_output_time > self.gpt_output_expiry:
            self.pub.publish(self.gpt_output)
            #rospy.loginfo(f"パブリッシュ: {self.gpt_output}")

    def run(self):
        """
        メインのループ処理。
        """
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            # 停止時間が一定以上の場合、GPTにリクエストを送信
            if self.stationary_time >= self.stationary_duration:
                # 前回のリクエストから十分な時間が経過している場合
                if rospy.get_time() - self.last_gpt_output_time > self.gpt_request_interval:
                    self.send_image_and_prompt()
            
            # 1秒ごとにパブリッシュを行う
            self.publish_gpt_output()
            
            # ループの速度を保つ
            rate.sleep()

if __name__ == "__main__":
    detector = ObstacleDetector()
    detector.run()




#!/usr/bin/env python3
# coding=utf-8

import rospy
import base64
import time
from dotenv import load_dotenv
import os
from os.path import join, dirname
from openai import OpenAI
from PIL import Image
import matplotlib.pyplot as plt
import japanize_matplotlib  # 日本語フォント対応

# .envファイルからAPIキーを読み込む
dotenv_path = join(dirname(__file__), '.env')
load_dotenv(dotenv_path)
API_KEY = os.environ.get("API_KEY")  # APIキーを取得

# APIキーが見つからない場合はエラーメッセージを表示して終了
if not API_KEY:
    print("APIキーが見つかりません。.envファイルに設定してください。")
    exit(1)

# OpenAIクライアントの初期化
client = OpenAI(api_key=API_KEY)

# ROSノードを初期化
rospy.init_node('gpt4o_image_request_node', anonymous=True)


# 画像をBase64でエンコードする関数
def encode_image(image_path):
    """
    指定された画像ファイルをBase64エンコードして文字列を返す。
    :param image_path: 画像ファイルのパス
    :return: Base64エンコードされた文字列
    """
    try:
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode('utf-8')
    except Exception as e:
        print(f"画像エンコード中にエラーが発生しました: {e}")
        return None

# テキストを指定文字数ごとに改行する関数
def split_text_by_length(text, length=30):
    """
    テキストを指定された文字数ごとに改行する。
    :param text: 元のテキスト
    :param length: 1行に表示する最大文字数
    :return: 改行されたテキスト
    """
    return '\n'.join([text[i:i+length] for i in range(0, len(text), length)])

# 画像を表示し、結果を表示する関数
def display_image_with_results(image_path, response_text, stats):
    """
    画像を表示し、その下に応答内容と統計情報を表示する。
    :param image_path: 表示する画像ファイルのパス
    :param response_text: GPT-4Oからの応答内容
    :param stats: 統計情報（処理時間など）
    """
    # 画像を読み込む
    img = Image.open(image_path)
    
    # 表示
    plt.figure(figsize=(8, 10))  # 画像とテキストの比率を調整
    plt.imshow(img)
    plt.axis('off')  # 画像の軸を非表示にする

    # 結果をテキストで表示
    wrapped_text = split_text_by_length(response_text, 30)
    plt.figtext(0.5, 0.01, f"Response:\n{wrapped_text}\n\nStats:\n{stats}",
                wrap=True, horizontalalignment='center', fontsize=10)
    plt.show()

# GPT-4Oに画像とテキストプロンプトを送信する関数
def send_image_and_prompt(image_path, prompt_text):
    """
    画像（Base64エンコード）とテキストプロンプトをGPT-4Oに送信する関数。
    処理時間を計算する。
    :param image_path: 画像ファイルのパス
    :param prompt_text: テキストプロンプト
    """
    # 画像をBase64でエンコード
    base64_image = encode_image(image_path)
    if not base64_image:
        print("画像のエンコードに失敗しました。")
        return

    # リクエストの作成
    try:
        start_time = time.time()  # 処理開始時刻を記録

        # APIリクエストを送信
        response = client.chat.completions.create(
            model="gpt-4o-mini",  # モデルの指定
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt_text},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{base64_image}"
                            },
                        },
                    ],
                }
            ],
        )

        # 処理終了時刻を記録
        end_time = time.time()

        # 応答内容を取得
        response_message = response.choices[0].message.content
        response_text = response_message.encode("utf-8").decode("utf-8") if response_message else "No content returned"

        # 統計情報を取得
        processing_time = f"{end_time - start_time:.2f} 秒"

        # 統計情報の文字列を作成
        stats = (f"処理時間: {processing_time}")

        # 結果を表示
        display_image_with_results(image_path, response_text, stats)

    except Exception as e:
        print(f"リクエスト中にエラーが発生しました: {e}")

# メイン処理
if __name__ == "__main__":
    # 使用例
    image_path = "/home/nakaba/bag/image_1732621413.jpeg"  # 画像ファイルのパスを指定
    prompt_text = "あなたはロボットオペレータです。ロボットが前方１ｍ以内に障害物を検出して止まっており、その処理を考えてください\
        ロボットのセンサは必ずしも１００％の信頼性があるわけではないため、誤検知の可能性もあります。\
        ロボットの前方に取り付けられたカメラの映像をもとにロボットの次の動作を判断し、\
        次のどのような行動を起こすべきかを１行目に数字だけで回答してください。\
        誤検知の可能性が高く、特に障害物がないので前に進んでも良さそうな場合は0,\
        障害物が前方にありそう、かつ、それが人やロボット等の移動障害物の場合にはしばらく待機して相手の移動を待つ戦略を取り、数値としては1、\
        カラーコーン等の静止障害物の場合は、回避する必要があるため2を出力し、改行して理由を出力してください\
        その際、わかるようであれば右か左かどちら方向に回避すればよりスムーズに回避できるかも教えてください"  
    send_image_and_prompt(image_path, prompt_text)