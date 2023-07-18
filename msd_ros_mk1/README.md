# msd_ros_mk1

## 1. 概要
MSD700のオリエンタルモータをRS485ModbusRTU通信を使用して制御する．
オリエンタルモータの用意したROSノードを利用して，ROSエコシステムによるモータ制御と自立移動を可能にした．

このROSパッケージの使用方法について説明する．

### 1.1 環境
- jetson nano B01
- Orientalmotor BLVM620KM-GFS x2
    - ドライバ品名：BLVD20KM
    - 出力：200W
- RS485コンバータ   [Contec COM-1PD(USB)H](https://www.contec.com/jp/products-services/daq-control/pc-helper/usb-module/com-1pd(usb)h/feature/)
- RJ45変換器    [StarTech.com DB9 - RJ45変換アダプタ DB-9](https://amzn.asia/d/8CeSP4T)
- [LANケーブル](https://www.amazon.co.jp/gp/product/B07ST8Q7VK/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&psc=1) x2

## 2. セットアップ
### 2.1 コンバータと変換器の接続
<!-- 配線接続の組み合わせの写真 -->

### 2.2 モータドライバのスイッチの設定
<!-- ディップスイッチの前後の写真 -->

## 3. 使い方
### 3.1 モータノードの起動

モータの電源を入れ，USB接続の確認をする．`ttyUSB0`があることを確認する．
```
ls /dev/tty*
```

もし無ければ，以下の操作をする．確認できた場合はこれをスキップして次に進んでください．
```
sudo modprobe ftdi_sio
sudo chmod 666 /sys/bus/usb-serial/drivers/ftdi_sio/new_id 
sudo echo 06ce 8331 > /sys/bus/usb-serial/drivers/ftdi_sio/new_id
```

モータノードをlaunchで起動．このとき，om_modbus_masterとom_motor.pyが起動している．
```
roslaunch msd_ros_mk1 msd700.launch
```

### 3.2 コントローラノードの起動

joyスティックを使用する場合
```
roslaunch msd_ros_mk1 msd700.launch
```


キーボードを使用する場合
```
roslaunch msd_ros_mk1 msd700.launch controller:=key
```