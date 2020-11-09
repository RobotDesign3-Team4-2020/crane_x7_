#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("crane_x7_pick_and_place_controller")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    gripper = moveit_commander.MoveGroupCommander("gripper")

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)

    print("---起動")
    rospy.sleep(1.0)

    # 何かを掴んでいた時のためにハンドを開く
    gripper.set_joint_value_target([0.9, 0.9])
    gripper.go()

    # SRDFに定義されている"home"の姿勢にする
    arm.set_named_target("home")
    arm.go()

    # ハンドを少し閉じる
    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()

    waypoints = []

    target_pose = arm.get_current_pose().pose

    # 上方から掴みに行く場合
    q = quaternion_from_euler(-3.14/2.0, 0.0, -3.14/2.0)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    waypoints.append(copy.deepcopy(wpose)) # Waypointの追加

    print("ウンコの消毒を開始")
    rospy.sleep(1.0)
    print("Step.1　ウンコを掴む")
    rospy.sleep(1.0)

    target_pose.position.x = 0.1
    target_pose.position.y = -0.3
    target_pose.position.z = 0.07
    waypoints.append(copy.deepcopy(wpose))

    rospy.sleep(1.5)

    wpose.position.x += 0.15
    waypoints.append(copy.deepcopy(wpose))

    gripper.set_joint_value_target([0.3, 0.3])
    gripper.go()

    print("Step.2　ウンコを移動させる")
    rospy.sleep(1.0)

    wpose.position.z += 0.1
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y += 0.45
    waypoints.append(copy.deepcopy(wpose))

    rospy.sleep(0.5)

    wpose.position.z -= 0.1
    waypoints.append(copy.deepcopy(wpose))

    gripper.set_joint_value_target([0.7, 0.7])
    gripper.go()

    rospy.sleep(1.0)

    wpose.position.x -= 0.05
    waypoints.append(copy.deepcopy(wpose))

    arm.set_named_target("home")
    arm.go()

    print("Step.3　ウンコを消毒する")
    rospy.sleep(1.0)

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.15
    target_pose.position.y = 0.2
    target_pose.position.z = 0.28
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    arm_initial_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_initial_pose)

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.25
    target_pose.position.y = 0.2
    target_pose.position.z = 0.28
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    gripper.set_joint_value_target([0.2, 0.2])
    gripper.go()

    rospy.sleep(1.0)

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.25
    target_pose.position.y = 0.2
    target_pose.position.z = 0.23
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.25
    target_pose.position.y = 0.2
    target_pose.position.z = 0.28
    q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]
    arm.set_pose_target(target_pose)  # 目標ポーズ設定
    arm.go()  # 実行

    arm.set_named_target("home")
    arm.go()

    print("消毒完了.　ウンコは綺麗になりました")
    rospy.sleep(1.5)
    print("---終了")

if __name__ == '__main__':

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass