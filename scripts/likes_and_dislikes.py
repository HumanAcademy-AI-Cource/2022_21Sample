#!/usr/bin/env python3

# ライブラリのインポート
import rospy
import roslib.packages
import cv2
import time
import random
import boto3
import wave
import subprocess
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class LikesAndDislikes():
    def __init__(self):
        # サブスクライバーを定義
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        # 画像を保持する変数
        self.image = None
        # 画像の保存先
        self.image_path = roslib.packages.get_pkg_dir("ai_integration_01") + "/scripts/camera.jpg"
        # 音楽ファイルの保存先
        self.audio_path = roslib.packages.get_pkg_dir("ai_integration_01") + "/scripts/speech.wav"

        # AWSと通信するための窓口を作成
        self.rekognition_client = boto3.client(service_name="rekognition")
        self.polly_client = boto3.client(service_name="polly")
        self.translate_client = boto3.client(service_name="translate")

    # カメラ画像を受け取るコールバック関数
    def image_callback(self, msg):
        # 画像をROSのデータ形式からOpenCV形式に変換して変数に保存
        self.image = CvBridge().imgmsg_to_cv2(msg, "bgr8")

    # オーディオデータか音楽ファイルを作成する関数
    def makeAudioFile(self, path, data):
        # 引数: 音楽ファイルを保存するパス, オーディオデータ
        # オーディオデータから音楽ファイル作成
        wave_data = wave.open(path, "wb")
        wave_data.setnchannels(1)
        wave_data.setsampwidth(2)
        wave_data.setframerate(16000)
        wave_data.writeframes(data)
        wave_data.close()

    # Pollyを使って音声合成して再生する関数
    def speechPolly(self, speech_text):
        print("発話させる文章: {}".format(speech_text))
        # 音声合成開始
        response_data = self.polly_client.synthesize_speech(
            Text=speech_text, OutputFormat="pcm", VoiceId="Mizuki"
        )
        audio_data = response_data["AudioStream"]
        # 音声合成のデータを音楽ファイル化
        self.makeAudioFile(self.audio_path, audio_data.read())
        # 保存したWAVデータを再生
        subprocess.check_call("aplay -D plughw:Headphones {}".format(self.audio_path), shell=True)

    def translate(self, text, source, target):
        # 翻訳開始
        response_data = self.translate_client.translate_text(
            Text=text,
            SourceLanguageCode=source,
            TargetLanguageCode=target
        )
        # レスポンスデータから「translated_text」を取り出す
        translated_text = response_data["TranslatedText"]
        return translated_text

    def run(self):
        # 一定周期で処理を実行するための準備
        rate = rospy.Rate(1)
        # ループ処理開始
        rospy.loginfo("AWSを使った画像認識を開始")
        while not rospy.is_shutdown():
            if self.image is not None:
                # カメラ画像を保存
                cv2.imwrite(self.image_path, self.image)
                # launchファイルに書かれている翻訳をするかどうかの設定を読み込む
                is_transrate = rospy.get_param("likes_and_dislikes_node/transrate")

                # 画像を開く
                with open(self.image_path, "rb") as f:
                    # 画像を読み込んで、変数に格納
                    image = f.read()

                    # Rekognitionを使ってモノや風景を認識
                    try:
                        response_data = self.rekognition_client.detect_labels(
                            Image={"Bytes": image}
                        )
                    except Exception as e:
                        print("AWSが混み合っていますので、しばらくお待ちください。")
                        # ランダムに待機時間を設定
                        time.sleep(int(random.uniform(0, 5)))
                        continue

                    # レスポンスデータから「Labels」を取り出す
                    labels = response_data["Labels"]
                    # 認識したラベルの名称を保存しておくリスト
                    name_list = []
                    # 認識したラベルをfor文で取り出す
                    for label in labels:
                        # ラベルデータからラベルの名称を取り出す
                        name = label["Name"]
                        # 結果の信頼度（どのくらい答えに自信があるか）を取り出す
                        confidence = label["Confidence"]
                        # 取り出した名称をリストに保存しておく
                        name_list.append(name)

                        # 翻訳モードが有効(True)の場合はラベルの名称を翻訳して表示する
                        if is_transrate:
                            print("ラベル: {}({}) - {:.1f}%".format(name, self.translate(name, "en", "ja"), confidence))
                        else:
                            print("ラベル: {} - {:.1f}%".format(name, confidence))

                    print("------------------------------------------")

                    # --------------------------------------------------
                    # バナナとナスにだけ好き嫌いのリアクションをする処理
                    # --------------------------------------------------
                    detect_flag = False
                    for name in name_list:
                        if name == "Banana":
                            self.speechPolly("ばなな大好き！！！")
                            detect_flag = True
                        elif name == "Eggplant":
                            self.speechPolly("ナスきらい！！！")
                            detect_flag = True

                    if not detect_flag:
                        rospy.loginfo("バナナとナスが見つかりませんでした。")

                    print("------------------------------------------")
            rate.sleep()


if __name__ == "__main__":
    try:
        # ノードを宣言
        rospy.init_node("likes_and_dislikes_node")
        # クラスのインスタンスを作成
        likes_and_dislikes = LikesAndDislikes()
        # 処理開始
        likes_and_dislikes.run()
    except rospy.ROSInterruptException:
        pass
