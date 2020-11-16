[English](README.en.md) | [日本語](README.md)
# うんこ消毒ロボット

## CRANE-X7のROSパッケージをインストール
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/RobotDesign3-Team4-2020/crane_x7_ros.git
$ rosdep install -r -y --from-paths --ignore-src crane_x7_ros
```
## branchの移動
```
cd ~/catkin_ws/src/crane_x7_ros
git checkout unko_shodoku_try
```
## 実機を使う場合
### モデル配置
- 用意するもの
  - うんこのおもちゃ
  - 市販の据え置き型アルコールスプレー  
<img src= "https://github.com/RobotDesign3-Team4-2020/interim_report/blob/master/img/img4.jpg" width="400">

- 配置方法
<img src= "https://github.com/RobotDesign3-Team4-2020/interim_report/blob/master/img/img5.png" width="400">  
<img src= "https://github.com/RobotDesign3-Team4-2020/interim_report/blob/master/img/img2.jpg" width="400">  
<img src= "https://github.com/RobotDesign3-Team4-2020/interim_report/blob/master/img/img1.jpg" width="400">  

### 実機の動かし方
動作確認する場合、信号ケーブルを接続した状態で次のコマンドを実行してください。
```
$ sudo chmod 666 /dev/ttyUSB0
$ roslaunch crane_x7_bringup demo.launch fake_execution:=false  
```  
## gazeboを使用する場合 
   
   - gazeboの起動方法  
   ~~~
   $ roslaunch crane_x7_gazebo crane_x7_with_table.launch
   ~~~
## プログラム
 - うんこを消毒する手順   
1.うんこをつかむ  
2.うんこを消毒スプレーのそばに置く  
3.消毒スプレーを押す

  - 実行方法  
  下記のコマンドを実行してください  
  ~~~
rosrun crane_x7_examples try_third.py
  ~~~
  - ソースコードは[こちら](https://github.com/RobotDesign3-Team4-2020/crane_x7_ros/blob/master/crane_x7_examples/scripts/try_third.py)をご覧ください
