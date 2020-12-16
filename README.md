# うんこ消毒ロボット
## CRANE-X7のROSパッケージをインストール
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/RobotDesign3-Team4-2020/crane_x7_.git
$ rosdep install -r -y --from-paths --ignore-src crane_x7_
```
## branch移動
```
$ cd ~/catkin_ws/src/
$ git checkout R.kamioka
```
## RealSense D435iのインストール
####RealSense SDKのインストール
サーバーの公開鍵を登録
`$ sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE`
サーバーをリポジトリのリストに追加（Ubuntu 18 LTS以外はことなるので注意）
`$ sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u`
更新
`$ sudo apt update`
ライブラリのインストール
`$ sudo apt install librealsense2-dkms`
`$ sudo apt install librealsense2-utils`
オプションで、開発者パッケージとデバッグパッケージをインストール
`$ sudo apt install librealsense2-dev`
`$ sudo apt install librealsense2-dbg`
Realsenseを再接続して実行確認
`$ realsense-viewer`

####ROS Wrapperのインストール
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
$ git clone https://github.com/IntelRealSense/realsense-ros.git
$ cd ~/catkin_ws
$catkin_make
```

####注意点
- WSL2でUSBシリアルの使用は難しかったです

詳しくは公式のリポジトリを参考にしてください。  
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
https://github.com/IntelRealSense/realsense-ros

## RealSEnse D435iをcrane_x7に取り付ける

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
