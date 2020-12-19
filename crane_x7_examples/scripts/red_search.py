#!/usr/bin/env python
# coding: utf-8
#searching pose

import rospy
import time
import actionlib
import cv2
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
import geometry_msgs.msg 
import moveit_commander
import rosnode
from tf.transformations import quaternion_from_euler, euler_from_quaternion 
from control_msgs.msg import (      
    GripperCommandAction,
    GripperCommandGoal
 )
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import sys
import numpy as np
from geometry_msgs.msg import Pose2D

global s_x , s_y
global flag
flag=0

class ArmJointTrajectoryExample(object):
    def __init__(self):
        self._client = actionlib.SimpleActionClient(
            "/crane_x7/arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        rospy.sleep(0.1)
        if not self._client.wait_for_server(rospy.Duration(secs=5)):
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server Not Found")
            sys.exit(1)
        self.gripper_client = actionlib.SimpleActionClient("/crane_x7/gripper_controller/gripper_cmd",GripperCommandAction)
        self.gripper_goal = GripperCommandGoal()
        # Wait 5 Seconds for the gripper action server to start or exit
        self.gripper_client.wait_for_server(rospy.Duration(5.0))
        self.num = 0
        if not self.gripper_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)
        #self.sub = rospy.Subscriber("bool", Pose2D, self.callback)
         
         
    global sum_x
    sum_x = 0
    global sum_y
    sum_y = 0
    global s_sum_x
    s_sum_x = 0
    global s_sum_y
    s_sum_y = 0
    global count
    count = 0
    global s_count
    s_count = 0
    global x
    x = 0
    global y
    y = 0
    global z
    z = 0
    global alc_x
    alc_x = 0
    global alc_y
    alc_y = 0
    img = cv2.imread('opencv_logo.png',0)
           
    def callback(self, ms):
        global mode
        global count
        global s_count
        global rad
        global angle
        if count >250:
            if s_count < 500:
                if s_count >= 300: #ms.theta == 1 no notoki kiiro naiyo
                    global s_sum_x
                    s_sum_x += ms.x #軸変換(センサのy軸をマニピュレータのx軸に合わせる)
                    global s_sum_y
                    s_sum_y += ms.y #軸変換(センサのx軸をマニピュレータのy軸に合わせる)
                s_count+=1    
            else:
                self.calculate(angle,alc_x,alc_y,s_sum_x/200,s_sum_y/200) #平均取ったものをself.calculateを送る
            
        elif ms.theta >= 1: #ms.theta == 1 no notoki kiiro naiyo 
            if count < 200:
                rad = ms.theta
                if rad > 1:
                    if rad < 50:
                        angle = rad
                if rad > 150:
                    if rad < 180:
                        angle = -(180-rad)
                global sum_x
                sum_x += ms.x #軸変換(センサのy軸をマニピュレータのx軸に合わせる)
                global sum_y
                sum_y += ms.y #軸変換(センサのx軸をマニピュレータのy軸に合わせる)
                count += 1
            else:
                print(angle)
                global alc_x
                alc_x=sum_x/350
                global alc_y
                alc_y=sum_y/350
                
                count+=100
                
                robot = moveit_commander.RobotCommander()
                arm = moveit_commander.MoveGroupCommander("arm")
                arm.set_max_velocity_scaling_factor(0.1)
                gripper = moveit_commander.MoveGroupCommander("gripper")
                target_pose = geometry_msgs.msg.Pose()
                target_pose.position.x = 0
                target_pose.position.y = -0.3
                target_pose.position.z = 0.3
                q = quaternion_from_euler(-3.14, 0.0, 3.14/2.0)  # 上方から掴みに行く場合
                target_pose.orientation.x = q[0]
                target_pose.orientation.y = q[1]
                target_pose.orientation.z = q[2]
                target_pose.orientation.w = q[3]
                arm.set_pose_target(target_pose)  # 目標ポーズ設定
                arm.go()  # 実行
                
                #self.calculate(angle,sum_x/350,sum_y/350) #平均取ったものをself.calculateを送る
                flag = 1
                print("ave_x :{}".format(sum_x/350))
                print("ave_y :{}".format(sum_y/350))
                print("theta")
        else :
            print("-----------------\nCan't find unko\n----------------------")
            self.num +=1  
            print(ms.theta)

        print("count = {}".format(count))
        print("s_count = {}".format(s_count))


    def callback2(self, angle,tar_x,tar_y,s_tar_x,s_tar_y):
        radian_angle = angle * 3.14/180    
        self.pick_and_put(radian_angle,tar_x,tar_y,s_tar_x,s_tar_y)


    def pick_and_put(self,radian_angle,tar_x,tar_y,s_tar_x,s_tar_y):
        robot = moveit_commander.RobotCommander()
        arm = moveit_commander.MoveGroupCommander("arm")
        arm.set_max_velocity_scaling_factor(0.1)
        gripper = moveit_commander.MoveGroupCommander("gripper")
        # 何かを掴んでいた時のためにハンドを開く
        gripper.set_joint_value_target([0.9, 0.9])
        gripper.go()
        # 掴む準備をする
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = s_tar_x
        target_pose.position.y = s_tar_y
        target_pose.position.z = 0.3
        q = quaternion_from_euler(-3.14, 0.0, 3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

        # ハンドを開く
        gripper.set_joint_value_target([0.7, 0.7])
        gripper.go()

        # 掴みに行く
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = s_tar_x
        target_pose.position.y = s_tar_y
        target_pose.position.z = 0.1
        q = quaternion_from_euler(-3.14, 0.0, 3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()  # 実行

        # ハンドを閉じる
        gripper.set_joint_value_target([0.04, 0.04])
        gripper.go()

        arm.set_named_target("vertical")
        arm.go()
        # SRDFに定義されている"landing"の姿勢にする
        arm.set_named_target("landing")
        arm.go()
        
        
        target_pose.position.x = tar_x+(0.12*math.sin(radian_angle))
        target_pose.position.y = tar_y-(0.12*math.cos(radian_angle))
        if math.sin(radian_angle)<0:
            target_pose.position.y -= 0.04 
            target_pose.position.x -= 0.02
            tar_x -=0.02
            tar_y -=0.04
        target_pose.position.z = 0.14
        q = quaternion_from_euler(-3.14, 0.0, radian_angle)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()
        
        # ハンドを開く
        gripper.set_joint_value_target([0.8, 0.8])
        gripper.go()
        
        target_pose.position.x = tar_x+(0.12*math.sin(radian_angle))
        target_pose.position.y = tar_y-(0.12*math.cos(radian_angle))
        if math.sin(radian_angle)<0:
            target_pose.position.y -= 0.04 
            target_pose.position.x -= 0.02
            tar_x -=0.02
            tar_y -=0.04
        target_pose.position.z = 0.4
        q = quaternion_from_euler(-3.14, 0.0, radian_angle)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()
        
        # SRDFに定義されている"landing"の姿勢にする
        arm.set_named_target("vertical")
        arm.go()

        self.angle_change(radian_angle,tar_x,tar_y)




    def angle_change(self, radian_angle,tar_x,tar_y):
        robot = moveit_commander.RobotCommander()
        arm = moveit_commander.MoveGroupCommander("arm")
        arm.set_max_velocity_scaling_factor(0.1)
        gripper = moveit_commander.MoveGroupCommander("gripper")


        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(0.5)
        
        target_pose = geometry_msgs.msg.Pose()
        
        
        #押す前の位置、姿勢 
        target_pose.position.x = tar_x-0.02
        target_pose.position.y = tar_y
        target_pose.position.z = 0.33
        q = quaternion_from_euler(-3.14, 0.0, radian_angle)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1.0)
        
        #押す前の位置、姿勢 
        target_pose.position.x = tar_x-0.02
        target_pose.position.y = tar_y
        target_pose.position.z = 0.215
        q = quaternion_from_euler(-3.14, 0.0, radian_angle)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1.0)
        
        target_pose.position.x = tar_x-0.02
        target_pose.position.y = tar_y
        target_pose.position.z = 0.33
        q = quaternion_from_euler(-3.14, 0.0, radian_angle)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(1.0)
        
        #homeに
        arm.set_named_target("vertical")
        arm.go()


    def calculate(self,angle, ave_x, ave_y,s_ave_x,s_ave_y):
        
        print("ave_x:{}".format(ave_x))
        print("ave_y:{}".format(ave_y))
        rad_y = 18  
        rad_x = rad_y * 1.33
        res_x = z * math.tan(math.radians(rad_x)) * ave_x / 320  #座標変換(画像の座標からセンサから見た実際の座標へ)
        #res_x = z * math.tan(math.radians(34.7)) * ave_x / 320  #座標変換(画像の座標からセンサから見た実際の座標へ)
        res_y = z * math.tan(math.radians(rad_y)) * ave_y / 240 #座標変換(画像の座標からセンサから見た実際の座標へ)
        s_res_x = z * math.tan(math.radians(rad_x)) * s_ave_x / 320  #座標変換(画像の座標からセンサから見た実際の座標へ)
        #res_x = z * math.tan(math.radians(34.7)) * ave_x / 320  #座標変換(画像の座標からセンサから見た実際の座標へ)
        s_res_y = z * math.tan(math.radians(rad_y)) * s_ave_y / 240 #座標変換(画像の座標からセンサから見た実際の座標へ)
        print("calculated\n res_x:{}".format(res_x))
        print("res_y:{}".format(res_y))

        self.pick(angle,res_x, res_y,s_res_x, s_res_y)

    def search(self, num):
        robot = moveit_commander.RobotCommander()
        arm = moveit_commander.MoveGroupCommander("arm")
        arm.set_max_velocity_scaling_factor(0.1)
        gripper = moveit_commander.MoveGroupCommander("gripper")

        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        rospy.sleep(1.0)

        print("Group names:")
        print(robot.get_group_names())

        print("Current state:")
        #print(robot.get_current_state())

        # アーム初期ポーズを表示
        arm_initial_pose = arm.get_current_pose().pose
        print("Arm initial pose:")
        print(arm_initial_pose)

        # 何かを掴んでいた時のためにハンドを開く
        gripper.set_joint_value_target([0.9, 0.9])
        gripper.go()

        # SRDFに定義されている"home"の姿勢にする
        arm.set_named_target("vertical")
        arm.go()
        #gripper.set_joint_value_target([0.7, 0.7])
        #gripper.go()
          
        target_pose = geometry_msgs.msg.Pose()
        
        #つかむ前の姿勢
        search_x = 0.2
        search_y = 0.1
        search_z = 0.33
        global x
        x = search_x 
        global y
        y = search_y
        global z
        z = search_z 
        target_pose.position.x = search_x
        target_pose.position.y = search_y
        target_pose.position.z = search_z
        q = quaternion_from_euler(-3.14, 0, 3.14/2.0)  # 上方から掴みに行く場合
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        
        arm.set_pose_target(target_pose)  # 目標ポーズ設定
        arm.go()
        
        print("go end") 
        rospy.sleep(0.5)
        print("sleep end")
        global mode
        mode = 0 
        self.sub = rospy.Subscriber("bool", Pose2D, self.callback,)

    
           
    
    def pick(self, angle,res_x, res_y,s_res_x,s_res_y):
        print("sx~=====")
        print(s_res_x)
        print("sy~=====")
        print(s_res_y)
        print("x~=====")
        print(res_x)
        print("y~=====")
        print(res_y)
        robot = moveit_commander.RobotCommander()
        arm = moveit_commander.MoveGroupCommander("arm")
        arm.set_max_velocity_scaling_factor(0.1)
        gripper = moveit_commander.MoveGroupCommander("gripper")

        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(0.5)
        
        target_pose = geometry_msgs.msg.Pose()
        
        offset_x = 0.05            #カメラと手先のオフセット
        offset_y = 0.05            #カメラと手先のオフセット
        
        tar_x = res_y + x + offset_x                        #センサの位置にあった原点をマニピュレータの根元へ移動
        tar_y = -res_x + y + offset_y-0.02             #センサの位置にあった原点をマニピュレータの根元へ移動
        s_tar_x = s_res_y + offset_x                        #センサの位置にあった原点をマニピュレータの根元へ移動
        s_tar_y = -s_res_x -0.3 + offset_y-0.03          #センサの位置にあった原点をマニピュレータの根元へ移動
        
        arm.set_named_target("vertical")
        arm.go()


        #ハンドを閉じる
        gripper.set_joint_value_target([0.2,0.2])
        gripper.go()
        #global rad_s
        self.callback2(angle,tar_x,tar_y,s_tar_x,s_tar_y)


if __name__ == "__main__":
    rospy.init_node("arm_joint_trajectory_example")
    arm_joint_trajectory_example = ArmJointTrajectoryExample()
    arm_joint_trajectory_example.search(0)
    rospy.spin()
