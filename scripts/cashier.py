#!/usr/bin/env python3

# 必要なライブラリをインポート
import rospy
import cv2
import subprocess
import roslib.packages
import wave
import csv
import time
import random
import boto3
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Cashier(object):
    def __init__(self):
        # サブスクライバーを定義
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        # 画像を保持する変数
        self.image = None
        # 会計の合計額を記録する変数
        self.goukei = 0

        # 画像の保存先
        self.image_path = roslib.packages.get_pkg_dir("ai_integration_01") + "/scripts/camera.jpg"
        # 音楽ファイルの保存先
        self.audio_path = roslib.packages.get_pkg_dir("ai_integration_01") + "/scripts/speech.wav"

        # データベースファイルの場所
        self.database_path = roslib.packages.get_pkg_dir("ai_integration_01") + "/scripts/database.csv"

        # AWSと通信するための窓口を作成
        self.rekognition_client = boto3.client(service_name="rekognition")
        self.polly_client = boto3.client(service_name="polly")
        self.translate_client = boto3.client(service_name="translate")

        # データベースに使う辞書型変数
        self.database = {}

        # データベースを読み込む
        self.readDataBase()

        # 案内用のメッセージを表示
        self.infoMessage()

    # CSVファイルからデータベースを作成する
    def readDataBase(self):
        # CSVをファイルを開く
        with open(self.database_path, "r") as f:
            # CSVデータを読み取る
            csv_data = csv.reader(f)
            # for文でCSVデータを展開
            for i, row in enumerate(csv_data):
                # 一番最初のデータはヘッダーなのでスキップする
                if i == 0:
                    continue

                if len(row) > 0:
                    # モノの名前を取り出す
                    name = row[0]
                    # 価格を取り出す
                    price = row[1]
                    # 辞書型のデータとして記録
                    self.database[name] = price

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

    def speechPolly(self, speech_text, debug=False):
        if debug:
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

    # カメラ画像を受け取るコールバック関数
    def image_callback(self, msg):
        # 画像をROSのデータ形式からOpenCV形式に変換して変数に保存
        self.image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Camera", self.image)

        # 押されたキーを判定する
        press_key = cv2.waitKey(1)
        if press_key == ord("s"):
            # --------------------------------
            # スキャンモード
            # --------------------------------
            cv2.imwrite(self.image_path, self.image)

            # 画像を開く
            with open(self.image_path, "rb") as f:
                image = f.read()

                # Rekognitionを使ってモノを認識
                try:
                    response_data = self.rekognition_client.detect_labels(
                        Image={"Bytes": image}
                    )

                    # レスポンスデータから「Labels」を取り出す
                    labels = response_data["Labels"]

                    # 認識したモノの名称を取り出す
                    items = []
                    for label in labels:
                        # ラベルデータからラベルの名称を取り出す
                        name = label["Name"]
                        if name in self.database:
                            items.append((name, self.database[name]))

                    # データベースと一致するものが全くなかった場合
                    if len(items) == 0:
                        print("商品が見つかりませんでした。スタッフを呼んでください。")
                        print("=======================================================================")
                        # 案内文を音声合成して再生
                        self.speechPolly("商品が見つかりませんでした。スタッフを呼んでください。")
                    else:
                        # 会計対象のモノの名前を取り出す
                        name = items[0][0]
                        transrated_name = self.translate(name, "en", "ja")
                        price = items[0][1]

                        print("スキャンされた商品: {}(¥{})".format(transrated_name, price))
                        # 案内文を音声合成
                        self.speechPolly("{}がスキャンされました。価格は{}円です。".format(transrated_name, price))

                        # 合計金額を計算
                        self.goukei += int(price)
                    # 案内用のメッセージを表示
                    self.infoMessage()

                except Exception as e:
                    print("AWSが混み合っていますので、しばらくお待ちください。")
                    # ランダムに待機時間を設定
                    time.sleep(int(random.uniform(0, 5)))
                    print(e)
                    return

        if press_key == ord("e"):
            # 会計モード
            print("お会計は¥{}です。".format(self.goukei))
            print("お支払いは不要です。ご利用ありがとうございました！")
            self.speechPolly("お会計は¥{}です。お支払いは不要です。ご利用ありがとうございました！".format(self.goukei))
            print("=======================================================================")
            print("３秒後にプログラムを終了します。")
            rospy.sleep(3)
            # 終了処理
            rospy.signal_shutdown("Shutdown")

    def infoMessage(self):
        print("=======================================================================")
        print("無人レジシステム")
        print("  - カメラウィンドウを選択した状態で[s]キーを押すと商品スキャン")
        print("  - お会計は[e]キー")
        print("=======================================================================")


if __name__ == "__main__":
    try:
        # ノードを宣言
        rospy.init_node("cashier_node")
        # クラスのインスタンスを作成
        cashier = Cashier()
        # 処理開始
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
