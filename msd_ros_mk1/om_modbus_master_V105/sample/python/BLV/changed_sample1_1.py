#!/usr/bin/python
# -*- coding: utf-8 -*-

#
# 対象機種:BLV
# 処理内容1:運転データNo.2の回転速度を書き込み
#

# モジュールのインポート
import rospy
import time
from om_modbus_master.msg import om_query
from om_modbus_master.msg import om_response
from om_modbus_master.msg import om_state

# グローバル変数
gState_driver = 0   # 通信可能フラグ変数(0:通信可能,1:通信中)


def stateCallback(res):
    """ステータスコールバック関数

    購読したステータスデータをグローバル変数に反映する

    """
    global gState_driver
    gState_driver = res.state_driver


def wait():
    """処理待ちサービス関数

    規定時間後(30ms)、通信可能になるまでウェイトがかかるサービス

    """
    global gState_driver
    time.sleep(0.03)  # ウェイト時間の設定(1 = 1.00s)
    # 通信が終了するまでループ
    while (gState_driver == 1):
        pass


def main():
    """メイン関数

      処理内容1:運転データNo.2の回転速度を書き込み

    """
    rospy.init_node("sample1_1", anonymous=True)
    pub = rospy.Publisher("om_query1", om_query,
                          queue_size=1)  # OMにノードに送信するまでの定義

    rospy.Subscriber("om_state1", om_state, stateCallback)  # レスポンスのコールバック定義
    msg = om_query()  # ノードで定義されたメッセージを使用
    time.sleep(1)

# 運転指令(FWD方向)(M1,START/STOP,RUN/BRAKEをON)
    msg.slave_id = 0x01     # 号機選択(Hex): 1号機
    msg.func_code = 1       # ファンクションコード選択: 1(Write)
    msg.write_addr = 124    # 先頭アドレス選択(Dec): 動作コマンド
    msg.write_num = 1       # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 26        # 書き込みデータ: ONビット(0000 0000 0001 1010) = 26
    pub.publish(msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                  # 処理待ち

    print("start")

    # 書き込み(回転速度No.2)
    msg.slave_id = 0x01     # 号機選択(Hex): 1号機
    msg.func_code = 1       # ファンクションコード選択: 1(Write)
    msg.write_addr = 1156   # 先頭アドレス選択(Dec): データNo.2 回転速度
    msg.write_num = 1       # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 4000       # 回転速度[r/min]
    pub.publish(msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                  # 処理待ち

    time.sleep(3)

    print("stop")

    # 減速停止
    msg.slave_id = 0x01   # 号機選択(Hex): 1号機
    msg.func_code = 1     # ファンクションコード選択: 1(Write)
    msg.write_addr = 124  # 先頭アドレス選択(Dec): 動作コマンド
    msg.write_num = 1     # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 18      # 書き込みデータ: ONビット(0000 0000 0001 0010) = 18
    pub.publish(msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                # 処理待ち



    print("END")  # 終了表示
    rospy.spin()


if __name__ == '__main__':
    main()
