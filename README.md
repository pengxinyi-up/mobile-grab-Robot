# mobile-grab-Robot
* Simulation design and Realization of face recognition, voice control, autonomous movement, recognition and grasping robot based on ROS
* Thanks for the great work: [ORB-SLAM2](https://github.com/introlab/find-object), [Cube SLAM](https://github.com/procrob/face_recognition)and[古月居](https://www.guyuehome.com/)
* Video:[Bilibili](https://www.bilibili.com/video/BV1WK4y147Rw?spm_id_from=333.999.0.0)

### [详细内容，点此链接可以去我的`CSDN博客`!](https://blog.csdn.net/qq_37372155/category_9650566.html) 


![system_structure](https://raw.githubusercontent.com/pengxinyi-up/mobile-grab-Robot/master/photos/system_structure.png "系统结构") 
![simulation_model](https://raw.githubusercontent.com/pengxinyi-up/mobile-grab-Robot/master/photos/simulation_model.png "仿真模型") 
![hardware_system](https://raw.githubusercontent.com/pengxinyi-up/mobile-grab-Robot/master/photos/hardware_system.png "硬件系统") 

## Instalation
This instalation process is for catkin (ROS Indigo or newer version) Assuming that your catkin workspace is under ~/catkin_ws, if not replace ~/catkin_ws with appropriate location. 
```bash
cd ~/catkin_ws/src
git clone https://github.com/pengxinyi-up/mobile-grab-Robot
cd ~/catkin_ws
catkin_make
```


## 人脸识别：
1、启动usb摄像头
```bash
roslaunch usb_cam usb_cam-test.launch
```
2、启动识别服务
```bash
rosrun face_recognition Fserver
rosrun face_recognition Fclient
```
3、添加样本,训练样本，更新样本并不断识别
```bash
rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 2 "your_name"
rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 3 "none"
rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 1 "none" 
```
4、识别样本(单次/循环/退出)
```bash
rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 0 "none"
rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 1 "none" 
rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 4 "none" 
```
## 建图：
1、启动仿真环境和moveit
```bash
roslaunch mbot_gazebo mbot_with_arm_bringup_moveit.launch
```
2、启动建图
```bash
roslaunch mbot_navigation gmapping_demo.launch
```
3、启动键盘
```bash
roslaunch mbot_teleop mbot_teleop.launch
```
4、保存地图
```bash
rosrun map_server map_saver -f /tmp/pxy_warehouse
```

## 导航：
1、启动仿真环境和和moveit
```bash
roslaunch mbot_gazebo mbot_with_arm_bringup_moveit.launch
```
2、启动导航
```bash
roslaunch mbot_navigation nav_room.launch
```
3、启动语音
```bash
rosrun robot_voice iat_publish
rosrun robot_voice voice_assistant
```
4、语音导航
```bash
python ~/ws_control/src/mbot_navigation/scripts/nav.py
```
5、打印位姿
```bash
rqt_plot /odom/pose/pose/position/x:y /odom/pose/pose/orientation/
```

## 目标识别：
1、启动识别程序
```bash
roslaunch find_object_2d find_object_3d.launch
```

## 抓取：
1、运动规划
```bash
python ~/ws_control/src/marm_planning/scripts/moveit_ik_demo.py
```
2、打印位姿
```bash
rqt_plot /joint_states/position[0]:position[1]:position[2]:position[3]:position[4]:position[5]:position[6]
```

## Contact
[Xinyi Peng](https://pxy.netlify.app/), Email: pengxinyi_up@163.com



