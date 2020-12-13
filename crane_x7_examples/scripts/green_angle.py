#!/usr/bin/env python
# coding: utf-8
#searching pose

import rospy
import time
import actionlib
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
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import sys
import numpy as np
from geometry_msgs.msg import Pose2D


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
         

          
 
    def callback(self, ms):
        global radian_angle 
        global mode
        global count
        radian_angle = ms.theta * 3.14/180    
        print ms.theta
        if radian_angle < 2.4:
            if radian_angle > 0.8:
                self.pick(radian_angle)
    

    
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
        arm.set_named_target("home")
        arm.go()
        gripper.set_joint_value_target([0.7, 0.7])
        gripper.go()
          
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
        rospy.sleep(2.0)
        print("sleep end")
        global mode
        mode = 0 
        self.sub = rospy.Subscriber("bool", Pose2D, self.callback)
    
           
    
           
    
    def pick(self, angle):
        robot = moveit_commander.RobotCommander()
        arm = moveit_commander.MoveGroupCommander("arm")
        arm.set_max_velocity_scaling_factor(0.1)
        gripper = moveit_commander.MoveGroupCommander("gripper")


        while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
            rospy.sleep(1.0)
        
        target_pose = geometry_msgs.msg.Pose()
        
        offset_x = 0.05            #カメラと手先のオフセット
        offset_y = 0.03            #カメラと手先のオフセット
        
        #tar_x = res_y + x + offset_x                        #センサの位置にあった原点をマニピュレータの根元へ移動
        #tar_y = -res_x + y + offset_y             #センサの位置にあった原点をマニピュレータの根元へ移動
        
        #つかむ前の位置、姿勢 
        target_pose.position.x = 0.2
        target_pose.position.y = 0.1
        target_pose.position.z = 0.33
        q = quaternion_from_euler(-3.14, 0.0, angle)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()
        rospy.sleep(5.0)
       
        #つかむ前の位置、姿勢 
        target_pose.position.x = 0.2
        target_pose.position.y = 0.1
        target_pose.position.z = 0.33
        q = quaternion_from_euler(-3.14, 0.0, 0.0)
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target(target_pose)
        arm.go()  
        print("nyannnyann")
        

        #self.go(mode)


if __name__ == "__main__":
    rospy.init_node("arm_joint_trajectory_example")
    arm_joint_trajectory_example = ArmJointTrajectoryExample() 
    arm_joint_trajectory_example.search(0)
    rospy.spin()