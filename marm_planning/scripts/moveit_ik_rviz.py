#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_msgs.msg import  PlanningScene, ObjectColor

obj_pose = Pose()
R_FLAG = 0
Count_sleep = 0
reference_frame = 'base_footprint'

def obj_pose_get(data):
    global obj_pose
    obj_pose = data

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
        # 初始化需要使用move group控制的机械臂中的arm group

        rospy.Subscriber("obj_pose", Pose, obj_pose_get)


        # 初始化场景对象
        scene = PlanningSceneInterface()
        # 创建一个发布场景变化信息的发布者
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        # 创建一个存储物体颜色的字典对象
        self.colors = dict()
        # 等待场景准备就绪
        rospy.sleep(1)
        # 设置场景物体的名称
        table_id = 'table'
        box1_id = 'box1'
        box2_id = 'box2'
        # 移除场景中之前运行残留的物体
        scene.remove_world_object(table_id)
        scene.remove_world_object(box1_id)
        scene.remove_world_object(box2_id)
        rospy.sleep(1)

        obj_pose.position.x=0.4
        obj_pose.position.y=0.1
        obj_pose.position.z=0.25
        # 0.40  0.35 # 0.4-5
        # 0     0    # 0
        # 0.26  0.26 # 0
        # 设置table、box1和box2的三维尺寸
        table_size = [0.2, 0.3, 0.01]
        box1_size = [0.03, 0.001, 0.08]
        # box2_size = [0.03, 0.03, 0.05]
        table_pose = PoseStamped()
        table_pose.header.frame_id = reference_frame
        table_pose.pose.position.x = table_size[0]/2.0+obj_pose.position.x - 0.05
        table_pose.pose.position.y = obj_pose.position.y
        table_pose.pose.position.z = obj_pose.position.z - table_size[2] / 2.0 - box1_size[2]/2.0
        # table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = reference_frame
        box1_pose.pose.position.x = obj_pose.position.x
        box1_pose.pose.position.y = obj_pose.position.y
        box1_pose.pose.position.z = obj_pose.position.z
        # box1_pose.pose.orientation.w = 1.0   
        scene.add_box(box1_id, box1_pose, box1_size)

        # 将桌子设置成红色，两个box设置成橙色
        self.setColor(table_id, 0, 0, 0, 1)
        self.setColor(box1_id, 0.8, 0, 0, 1.0)
        self.setColor(box2_id, 0.8, 0, 0, 1.0)
        # self.setColor(table_id, 0.8, 0, 0, 1.0) red
        # self.setColor(box1_id, 0.8, 0.4, 0, 1.0) yellow
        # self.setColor(box1_id, 1, 1, 1, 1.0) white
        # self.setColor(sphere_id, 0, 0, 0, 1) black
        # 将场景中的颜色设置发布
        self.sendColors() 

    def setColor(self, name, r, g, b, a = 0.9):
        # 初始化moveit颜色对象
        color = ObjectColor()
        
        # 设置颜色值
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # 更新颜色字典
        self.colors[name] = color

    # 将颜色设置发送并应用到moveit场景当中
    def sendColors(self):
        # 初始化规划场景对象
        p = PlanningScene()

        # 需要设置规划场景是否有差异     
        p.is_diff = True
        
        # 从颜色字典中取出颜色设置
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # 发布场景物体颜色设置
        self.scene_pub.publish(p)

def do_it():
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')

        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('arm')
        gripper = MoveGroupCommander('gripper')
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_footprint'
        arm.set_pose_reference_frame(reference_frame)
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)

        # 设置每次运动规划的时间限制：5s
        arm.set_planning_time(10)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01) #0.01
        arm.set_goal_orientation_tolerance(0.1) #0.05
        gripper.set_goal_position_tolerance(0.01)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('up')
        print u"初始化机械臂位置和夹爪。。。。。"
        arm.go()
        rospy.sleep(3)
        gripper.set_joint_value_target([0.01])
        gripper.go()
        rospy.sleep(1)

        print("Recive_FLAG: %d " % R_FLAG)
        print("obj_pose.position.x: %f " % obj_pose.position.x)
        print("obj_pose.position.y: %f " % obj_pose.position.y)
        print("obj_pose.position.z: %f " % obj_pose.position.z)
        print("obj_pose.orientation.x: %f " % obj_pose.orientation.x)
        print("obj_pose.orientation.y: %f " % obj_pose.orientation.y)
        print("obj_pose.orientation.z: %f " % obj_pose.orientation.z)
        print("obj_pose.orientation.z: %f " % obj_pose.orientation.w)
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = obj_pose.position.x-0.03
        target_pose.pose.position.y = obj_pose.position.y
        target_pose.pose.position.z = obj_pose.position.z+0.0
        # target_pose.pose.orientation.x = obj_pose.orientation.x
        # target_pose.pose.orientation.y = obj_pose.orientation.y
        # target_pose.pose.orientation.z = obj_pose.orientation.z
        # target_pose.pose.orientation.w = obj_pose.orientation.w
        # target_pose.pose.orientation.x = 1
        # target_pose.pose.orientation.y = 0
        # target_pose.pose.orientation.z = 0
        # target_pose.pose.orientation.w = 0
        print u"第1步规划。。。。。"
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        # 规划运动路径
        traj = arm.plan() 
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(3)

        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = obj_pose.position.x+0.01
        target_pose.pose.position.y = obj_pose.position.y
        target_pose.pose.position.z = obj_pose.position.z+0.0
        print u"第2步规划。。。。。"
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        # 规划运动路径
        traj = arm.plan() 
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(3)


        # # 控制机械臂终端向右移动5cm
        # arm.shift_pose_target(0, 0.01, end_effector_link)
        # arm.go()
        # rospy.sleep(1)

        gripper.set_joint_value_target([0.05])
        print u"闭合夹爪。。。。。"
        gripper.go()
        rospy.sleep(1)

        # # # 控制机械臂终端反向旋转90度
        # # arm.shift_pose_target(3, -1.57, end_effector_link)
        # # arm.go()
        # # rospy.sleep(1)

        arm.set_named_target('up')
        print u"收起机械臂。。。。。"
        arm.go()
        rospy.sleep(3)

if __name__ == "__main__":

    MoveItIkDemo() 
    while R_FLAG == 0:
        do_it()
        # if obj_pose.position.x != 0:
        # R_FLAG=1
        rospy.sleep(2)
        Count_sleep = Count_sleep+1
        print("Count_sleep: %d " % Count_sleep)
        if Count_sleep>=1:

            # 关闭并退出moveit
            moveit_commander.roscpp_shutdown()
            moveit_commander.os._exit(0)
            break
