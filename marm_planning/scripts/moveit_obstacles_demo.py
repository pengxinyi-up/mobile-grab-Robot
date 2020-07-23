#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose

from std_msgs.msg import Float64MultiArray 

pos_aim = [0,0,0]


def callback(msgg): #msgg的名字随便定义,保持一致即可 
    global Recive_FLAG
    global Receive_count
    if Recive_FLAG==0:
        rospy.loginfo("I heared:%6.2f %6.2f %6.2f \n",msgg.data[0],msgg.data[1],msgg.data[2]) #数组的大小要与talker里面的一样 
        pos_aim[0] = msgg.data[0]/100.0
        pos_aim[1] = msgg.data[1]/100.0
        pos_aim[2] = msgg.data[2]/100.0
        rospy.loginfo("X:%.2f Y:%.2f Z:%.2f \n",pos_aim[0],pos_aim[1],pos_aim[2])
        #if(++Receive_count>5):
        Recive_FLAG=1



class MoveItObstaclesDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_obstacles_demo')
        
        # 初始化场景对象
        scene = PlanningSceneInterface()
        
        # 创建一个发布场景变化信息的发布者
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        rospy.Subscriber("chatter", Float64MultiArray, callback) 
        
        # 创建一个存储物体颜色的字典对象
        self.colors = dict()
        
        # 等待场景准备就绪
        rospy.sleep(1)
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('arm')
        gripper = MoveGroupCommander('gripper')
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        gripper.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.2)
       
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
        
        # 设置每次运动规划的时间限制：5s
        arm.set_planning_time(5)
        
        # 设置场景物体的名称
        table_id = 'table'
        # cy_id = 'cy'
        box1_id = 'box1'
        box2_id = 'box2'
        box3_id = 'box3'
        sphere_id = 'sphere'
        
        # 移除场景中之前运行残留的物体
        scene.remove_world_object(table_id)
        scene.remove_world_object(box1_id)
        scene.remove_world_object(box2_id)    
        scene.remove_world_object(box3_id)
        scene.remove_world_object(sphere_id)
        rospy.sleep(1)
        
        #控制机械臂先回到初始化位置
        arm.set_named_target('init')
        arm.go()
        rospy.sleep(1)

        # arm.set_named_target('start')
        # arm.go()
        # rospy.sleep(1)

        gripper.set_joint_value_target([0.05])
        gripper.go()
        rospy.sleep(0)
        
        # 设置桌面的高度
        table_ground = 0.37
        
        # 设置table、box1和box2的三维尺寸
        table_size = [0.2, 0.3, 0.01]
        box1_size = [0.01, 0.01, 0.19]
        box2_size = [0.01, 0.01, 0.19]
        box3_size = [0.005, 0.01, 0.3]
        sphere_R = 0.01
        error = 0.03
        
        # 将三个物体加入场景当中
        table_pose = PoseStamped()
        table_pose.header.frame_id = reference_frame
        table_pose.pose.position.x = -table_size[0]/2.0
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)
        #scene.add_cylinder
        
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = reference_frame
        box1_pose.pose.position.x = -0.09
        box1_pose.pose.position.y = table_size[0]/2.0
        box1_pose.pose.position.z = 0.18 + box1_size[2]/2.0
        box1_pose.pose.orientation.w = 1.0   
        scene.add_box(box1_id, box1_pose, box1_size)
        
        box2_pose = PoseStamped()
        box2_pose.header.frame_id = reference_frame
        box2_pose.pose.position.x = -0.09
        box2_pose.pose.position.y = -table_size[0]/2.0
        box2_pose.pose.position.z = 0.18 + box1_size[2]/2.0
        box2_pose.pose.orientation.w = 1.0   
        scene.add_box(box2_id, box2_pose, box2_size)

        # box3_pose = PoseStamped()
        # box3_pose.header.frame_id = reference_frame
        # box3_pose.pose.position.x = pos_aim[0]
        # box3_pose.pose.position.y = pos_aim[1]
        # box3_pose.pose.position.z = box3_size[2]/2.0+0.1
        # box3_pose.pose.orientation.w = 1.0   
        # scene.add_box(box3_id, box3_pose, box3_size)

        sphere_pose = PoseStamped()
        sphere_pose.header.frame_id = reference_frame
        sphere_pose.pose.position.x = pos_aim[0]+0
        sphere_pose.pose.position.y = pos_aim[1]
        sphere_pose.pose.position.z = pos_aim[2]
        sphere_pose.pose.orientation.w = 1.0   
        scene.add_sphere(sphere_id, sphere_pose, sphere_R)
        
        # 将桌子设置成红色，两个box设置成橙色
        self.setColor(table_id, 0, 0, 0, 1)
        # self.setColor(table_id, 0.8, 0, 0, 1.0)
        # self.setColor(box1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box1_id, 1, 1, 1, 1.0)
        self.setColor(box2_id, 1, 1, 1, 1.0)
        self.setColor(box3_id, 1, 1, 1, 1.0)
        self.setColor(sphere_id, 0.8, 0, 0, 1.0)
        
        # 将场景中的颜色设置发布
        self.sendColors()   

        
        # rospy.INFO("waiting...")
        # if(Recive_FLAG==1): 
        #     rospy.INFO("OK！")

        # 设置机械臂的运动目标位置
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.pose.position.x = pos_aim[0]-error
        target_pose.pose.position.y = pos_aim[1]
        target_pose.pose.position.z = pos_aim[2]
        # target_pose.pose.orientation.w = 1.0
        #####0.3   
        # 0.2     0.2     0.2     0.15     0.15
        # 0.12    0.11    0.12    0.15     0.15
        # 0.30    0.245   0.35    0.35     0.25
        # 控制机械臂运动到目标位置
        arm.set_pose_target(target_pose, end_effector_link)
        arm.go()
        rospy.sleep(1)

        gripper.set_joint_value_target([0.03])
        gripper.go()
        rospy.sleep(1)


        # # 控制机械臂终端向x移动5cm
        arm.shift_pose_target(0, 0.01, end_effector_link)
        arm.go()
        rospy.sleep(1)

        gripper.set_joint_value_target([0.05])
        gripper.go()
        rospy.sleep(1)
    

        # # 设置机械臂的运动目标位置，进行避障规划
        # target_pose2 = PoseStamped()
        # target_pose2.header.frame_id = reference_frame
        # target_pose2.pose.position.x = 0.15
        # target_pose2.pose.position.y = 0 #-0.25
        # target_pose2.pose.position.z = table_pose.pose.position.z + table_size[2] + 0.05
        # target_pose2.pose.orientation.w = 1.0
        
        # # 控制机械臂运动到目标位置
        # arm.set_pose_target(target_pose2, end_effector_link)
        # arm.go()
        # rospy.sleep(2)
        
        #控制机械臂回到初始化位置
        arm.set_named_target('init')
        arm.go()
    
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

        rospy.spin()
        
    # 设置场景物体的颜色
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

if __name__ == "__main__":
    try:
        #rospy.init_node('listener', anonymous= True)
        
        
        Recive_FLAG = 0
        Receive_count = 0
        #while(Recive_FLAG<=1):
        MoveItObstaclesDemo()
        
    except KeyboardInterrupt:
        raise
    

    
