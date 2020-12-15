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
以下を参考にしてください。  
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
