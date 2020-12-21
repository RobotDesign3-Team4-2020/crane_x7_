# うんこ消毒ロボット
## CRANE-X7のROSパッケージをインストール
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/RobotDesign3-Team4-2020/crane_x7_.git
$ rosdep install -r -y --from-paths --ignore-src crane_x7_
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
