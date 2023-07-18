#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import math
from om_modbus_master.msg import om_query
from om_modbus_master.msg import om_response
from om_modbus_master.msg import om_state
from geometry_msgs.msg import Twist

gState_driver = 0   # 通信可能フラグ変数(0:通信可能,1:通信中)
gState_mes = 0      # メッセージ(0:メッセージなし,1:メッセージ到達,2:メッセージエラー)
gState_error = 0    # エラー(0:エラーなし,1:無応答,2:例外応答)
gMotor_spd1 = 0
gMotor_spd2 = 0
gLinear_x = 0.0
gAngular_z = 0.0

def cmdvelCallback(vel):
    global gLinear_x
    global gAngular_z
    f = 2
    gLinear_x =  round(vel.linear.x, f)
    gAngular_z = round(vel.angular.z, f)
    # rospy.loginfo('Speed = %s m/s, Angular = %s m/s', gLinear_x, gAngular_z)
    time.sleep(0.001)



class IK():
    def IK(self, v, w):
        D = 0.600       # 車輪間距離 [m]
        v_r = v + D * w
        v_l = v - D * w

        return v_l, v_r

    def velocityToRPM(self, v):
        REDUCTION_RATIO = 100        #reduction ratio # 減速比
        WHEEL_RADIUS = 0.1105        # Wheel_radius # 車輪半径[m]
        n = 30 / math.pi * REDUCTION_RATIO / WHEEL_RADIUS * v

        return n

    def IKExcution(self, x, z):
        left_motor = self.velocityToRPM(self.IK(x, z)[0])
        right_motor = self.velocityToRPM(self.IK(x, z)[1])
        list = [left_motor, right_motor]

        return list



def resCallback(res):
    global gMotor_spd1
    global gMotor_spd2
    if (res.slave_id == 1 and res.func_code == 3):
        gMotor_spd1 = res.data[0]
    elif (res.slave_id == 2 and res.func_code == 3):
        gMotor_spd2 = res.data[0]


def stateCallback(res):
    global gState_driver
    global gState_mes
    global gState_error
    gState_driver = res.state_driver
    gState_mes = res.state_mes
    gState_error = res.state_error


def wait():
    global gState_driver
    time.sleep(0.03)
    # ドライバの通信が終了するまでループ
    while (gState_driver == 1):
        pass


def init(msg, pub):
    # 運転入力方式の変更(3ワイヤ)
    msg.slave_id = 0x00     # 号機選択(Hex): 0(ブロードキャスト)
    msg.func_code = 1       # ファンクションコード選択: 1(Write)
    msg.write_addr = 4160   # 先頭アドレス選択(Dec): 運転入力方式パラメータ
    msg.write_num = 1       # 書き込みデータサイズ: 1(32bit)
    msg.data[0] = 1         # 書き込みデータ: 0(2ワイヤ),1(3ワイヤ)
    pub.publish(msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                  # 処理待ち

    # 運転データ 回展速度No.2を0[r/min]に初期化
    msg.slave_id = 0x00     # 号機選択(Hex): 0(ブロードキャスト)
    msg.func_code = 1       # ファンクションコード選択: 1(Write)
    msg.write_addr = 1156   # 先頭アドレス選択(Dec): データNo.2 回転速度
    msg.write_num = 1       # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 0         # 書き込みデータ: 0[r/min]
    pub.publish(msg)        # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                  # 処理待ち

    # Configrationの実行
    msg.slave_id = 0x00   # 号機選択(Hex): 0(ブロードキャスト)
    msg.func_code = 1     # ファンクションコード選択: 1(Write)
    msg.write_addr = 396  # 先頭アドレス選択(Dec): Configration実行コマンド
    msg.write_num = 1     # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = 1       # 書き込みデータ: 1(実行)
    pub.publish(msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                # 処理待ち


def directionDetect(left_rpm, right_rpm):
    if left_rpm > 0:
        left_direction = 26
        rospy.loginfo('Left Motor is FWD')
    elif left_rpm < 0:
        left_direction = 58
        rospy.loginfo('Left Motor is REV')
    else:
        left_direction = 58
        rospy.loginfo('Left Motor is Stop')

    if right_rpm > 0 :
        right_direction = 58
        rospy.loginfo('Right Motor is FWD')
    elif right_rpm < 0:
        right_direction = 26
        rospy.loginfo('Right Motor is REV')
    else:
        right_direction = 26
        rospy.loginfo('Right Motor is Stop')

    if left_rpm < 0 or right_rpm < 0 :
        right_rpm, left_rpm = left_rpm, right_rpm

    return left_direction, right_direction, left_rpm, right_rpm


def directionWrite(msg, pub, left_direction, right_direction):
    '''モータの正転は 26 ，逆転は 58 で設定される．左モータを正転に置いていると仮定すると，
       マシンが前進するとき，右モータは逆転する．
    '''
    # 運転指令(FWD方向)(M1,START/STOP,RUN/BRAKEをON)
    # 運転指令(REV方向)(M1,START/STOP,RUN/BRAKE,FWD/REVをON)
    msg.slave_id = 0x01               # 号機選択(Hex): 1号機
    msg.func_code = 1                 # ファンクションコード選択: 1(Write)
    msg.write_addr = 124              # 先頭アドレス選択(Dec): 動作コマンド
    msg.write_num = 1                 # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = left_direction      # 書き込みデータ: ONビット(0000 0000 0001 1010) = 26
    pub.publish(msg)                  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                            # 処理待ち

    msg.slave_id = 0x02               # 号機選択(Hex): 2号機
    msg.func_code = 1                 # ファンクションコード選択: 1(Write)
    msg.write_addr = 124              # 先頭アドレス選択(Dec): 動作コマンド
    msg.write_num = 1                 # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = right_direction     # 書き込みデータ: ONビット(0000 0000 0011 1010) = 58
    pub.publish(msg)                  # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                            # 処理待ち


def RPMLimit(rpm):     # 回転数制限 符号を外した後にこの判別を行い制限する
    if rpm >= 4000 :
        rpm = 4000
    elif rpm <= 100 :
        rpm = 0
    else:
        rpm = rpm
    return rpm


def stop(msg, pub):
    ''' 減速停止 18, 即時停止 10
    '''
    # x = 'botton pushed'
    # if x == 'botton pushed':
    #     stop_mode = 18
    # else:
    #     stop_mode = 10

    # 暫定定数
    stop_mode = 10

    msg.slave_id = 0x01   # 号機選択(Hex): 1号機
    msg.func_code = 1     # ファンクションコード選択: 1(Write)
    msg.write_addr = 124  # 先頭アドレス選択(Dec): 動作コマンド
    msg.write_num = 1     # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = stop_mode      # 書き込みデータ: ONビット(0000 0000 0001 0010) = 18
    pub.publish(msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                # 処理待ち
    
    msg.slave_id = 0x02   # 号機選択(Hex): 2号機
    msg.func_code = 1     # ファンクションコード選択: 1(Write)
    msg.write_addr = 124  # 先頭アドレス選択(Dec): 動作コマンド
    msg.write_num = 1     # 書き込みデータサイズ: 1 (32bit)
    msg.data[0] = stop_mode      # 書き込みデータ: ONビット(0000 0000 0010 1010) = 10
    pub.publish(msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()                # 処理待ち
    

def drive(msg, pub, left_rpm, right_rpm):
    global gMotor_spd1
    global gMotor_spd2

    if left_rpm and right_rpm != 0.0:
        rospy.loginfo(left_rpm)
        rospy.loginfo(right_rpm)
        # モータの回転方向を決定する
        rpm_direct = directionDetect(left_rpm, right_rpm)
        # モータの回転方向を書き込む
        directionWrite(msg, pub, rpm_direct[0], rpm_direct[1])

        ## 回転数を代入するために，絶対値に丸める処理

        left_rpm = RPMLimit(abs(rpm_direct[2]))
        right_rpm = RPMLimit(abs(rpm_direct[3]))

        # 回転速度の設定
        msg.slave_id = 0x01         # 号機選択(Hex): 1号機
        msg.func_code =1            # ファンクションコード選択: 1(Write)
        msg.write_addr = 1156       # 先頭アドレス選択(Dec): データNo.2 回転速度
        msg.write_num = 1           # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = left_rpm      # 書き込みデータ: [r/min]
        pub.publish(msg)            # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        wait()                      # 処理待ち
        # 回転速度の設定
        msg.slave_id = 0x02         # 号機選択(Hex): 2号機
        msg.func_code = 1           # ファンクションコード選択: 1(Write)
        msg.write_addr = 1156       # 先頭アドレス選択(Dec): データNo.2 回転速度
        msg.write_num = 1           # 書き込みデータサイズ: 1 (32bit)
        msg.data[0] = right_rpm     # 書き込みデータ: [r/min]
        pub.publish(msg)            # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
        wait()                      # 処理待ち

    else:
        stop(msg, pub)
        
    rospy.loginfo('Output Left motor = %s rpm, Right motor = %s rpm',left_rpm, right_rpm)


def feedbackRead(msg,pub):
    # 回転速度読み込み
    msg.slave_id = 0x01   # 号機選択(Hex): 1号機
    msg.func_code = 0     # ファンクションコード選択: 0(Read)
    msg.read_addr = 206   # 先頭アドレス選択(Dec): フィードバック速度[r/min](符号付)
    msg.read_num = 1      # 読み込みデータサイズ: 1 (32bit)
    pub.publish(msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()
    msg.slave_id = 0x02   # 号機選択(Hex): 2号機
    msg.func_code = 0     # ファンクションコード選択: 0(Read)
    msg.read_addr = 206   # 先頭アドレス選択(Dec): フィードバック速度[r/min](符号付)
    msg.read_num = 1      # 読み込みデータサイズ: 1 (32bit)
    pub.publish(msg)      # クエリ生成ノードに上記内容を送信。ノードでmsg作成後はドライバに送信
    wait()

    rospy.loginfo('FeedbackSpeed Left Motor %s Right Motor %s [r/min]', gMotor_spd1, gMotor_spd2)



def main():
    rospy.init_node("om_motor", anonymous=True)
    pub = rospy.Publisher("om_query1", om_query, queue_size=1)  # OMにノードに送信するまでの定義 抽出
    rospy.Subscriber("om_state1", om_state, stateCallback)      # レスポンスのコールバック定義
    rospy.Subscriber("om_response1", om_response, resCallback)  # レスポンスのコールバック定義
    msg = om_query()                                            # OMノードで作成したメッセージを使用
    time.sleep(1)                                               # 1秒待機
    init(msg, pub)                                              # 初期化関数のコール

    rospy.Subscriber("cmd_vel", Twist, cmdvelCallback)
    ik = IK()
        
    while not rospy.is_shutdown():
        vel_to_rpm = ik.IKExcution(gLinear_x, gAngular_z)
        left_rpm  = round(vel_to_rpm[0])
        right_rpm = round(vel_to_rpm[1])

        drive(msg, pub, left_rpm, right_rpm)

        feedbackRead(msg,pub)


if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException: pass