# うんこ消毒ロボット
## CRANE-X7のROSパッケージをインストール
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/RobotDesign3-Team4-2020/crane_x7_.git
$ rosdep install -r -y --from-paths --ignore-src crane_x7_
$ cd ~/catkin_ws && catkin_make
```

## RealSense D435iのインストール
#### RealSense SDKのインストール
サーバーの公開鍵を登録
```
$ sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
```
サーバーをリポジトリのリストに追加（Ubuntu 18 LTS以外はことなるので注意）
```
$ sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
```
更新
```
$ sudo apt update
```
ライブラリのインストール
```
$ sudo apt install librealsense2-dkms
$ sudo apt install librealsense2-utils
```
オプションで、開発者パッケージとデバッグパッケージをインストール
```
$ sudo apt install librealsense2-dev
$ sudo apt install librealsense2-dbg
```
Realsenseを再接続して実行確認
```
$ realsense-viewer
```

#### ROS Wrapperのインストール
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
$ git clone https://github.com/IntelRealSense/realsense-ros.git
$ cd ~/catkin_ws
$catkin_make
```

#### 注意点
- WSL2でUSBシリアルの使用は難しかったです

詳しくは公式のリポジトリを参考にしてください。

[Intel® RealSense™ SDK 2.0パッケージのインストール](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)

[ROS Wrapper for Intel® RealSense™ Devicesのインストール](https://github.com/IntelRealSense/realsense-ros)

## RealSense D435iをcrane_x7に取り付ける
以下の写真のように取り付ける

<img src="https://user-images.githubusercontent.com/70384485/102331893-c2444900-3fce-11eb-98f1-78d05cf59eff.png" width="320px">

## 消毒液のノズルにマーカーを取り付ける
以下の写真のように黄色で長方形のマーカーを取り付ける

<img src="https://user-images.githubusercontent.com/70384485/102332471-76de6a80-3fcf-11eb-9a8e-3b5462a1ad7b.png" width="320px">

## 実機の動かし方
動作確認する場合、信号ケーブルを接続した状態で次のコマンドを実行してください。
```
$ sudo chmod 777 /dev/ttyUSB0
$ roslaunch crane_x7_bringup demo.launch  
$ roslaunch realsense2_camera rs_camera.launch
$ rosrun crane_x7_robot_design3 cam
```


## プログラム
- 実行方法  
以下のコマンドを実行してください
```
$ rosrun crane_x7_examples red_search.py
```
------
## 知的財産権について
CRANE-X7は、アールティが開発した研究用アームロボットです。 このリポジトリのデータ等に関するライセンスについては、LICENSEファイルをご参照ください。 企業による使用については、自社内において研究開発をする目的に限り、本データの使用を許諾します。 本データを使って自作されたい方は義務ではありませんが弊社ロボットショップで部品をお買い求めいただければ、励みになります。 商業目的をもって本データを使用する場合は、商業用使用許諾の条件等について弊社までお問合せください。

サーボモータのXM540やXM430に関するCADモデルの使用については、ROBOTIS社より使用許諾を受けています。 CRANE-X7に使用されているROBOTIS社の部品類にかかる著作権、商標権、その他の知的財産権は、ROBOTIS社に帰属します。

## Proprietary Rights
CRANE-X7 is an arm robot developed by RT Corporation for research purposes. Please read the license information contained in this repository to find out more about licensing. Companies are permitted to use CRANE-X7 and the materials made available here for internal, research and development purposes only. If you are interested in building your own robot for your personal use by utilizing the information made available here, take your time to visit our website and purchase relevant components and parts – that will certainly help us keep going! Otherwise, if you are interested in manufacturing and commercializing products based on the information herein, please contact us to arrange a license and collaboration agreement with us.

We have obtained permission from ROBOTIS Co., Ltd. to use CAD models relating to servo motors XM540 and XM430. The proprietary rights relating to any components or parts manufactured by ROBOTIS and used in this product, including but not limited to copyrights, trademarks, and other intellectual property rights, shall remain vested in ROBOTIS.
