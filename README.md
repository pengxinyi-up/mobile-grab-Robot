# mobile-grab-Robot
Simulation design and Realization of face recognition, voice control, autonomous movement, recognition and grasping robot based on ROS

人脸识别：
1,启动usb摄像头
roslaunch usb_cam usb_cam-test.launch
2，启动识别服务
rosrun face_recognition Fserver
rosrun face_recognition Fclient
3，添加样本,训练样本，更新样本并不断识别
rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 2 "your_name"
rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 3 "none"
rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 1 "none" 
4，识别样本(单次/循环/退出)
rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 0 "none"
rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 1 "none" 
rostopic pub -1 /fr_order face_recognition/FRClientGoal -- 4 "none" 

建图：
1、启动仿真环境和moveit
roslaunch mbot_gazebo mbot_with_arm_bringup_moveit.launch
2,启动建图
roslaunch mbot_navigation gmapping_demo.launch
2、启动键盘
roslaunch mbot_teleop mbot_teleop.launch
3、保存地图
rosrun map_server map_saver -f /tmp/pxy_warehouse

导航：
1、启动仿真环境和和moveit
roslaunch mbot_gazebo mbot_with_arm_bringup_moveit.launch
2,启动导航
roslaunch mbot_navigation nav_room.launch
3，启动语音
rosrun robot_voice iat_publish
rosrun robot_voice voice_assistant
4，语音导航
python ~/ws_control/src/mbot_navigation/scripts/nav.py
5,打印位姿
rqt_plot /odom/pose/pose/position/x:y /odom/pose/pose/orientation/

目标识别：
1，启动识别程序
roslaunch find_object_2d find_object_3d.launch

抓取：
1，运动规划
python ~/ws_control/src/marm_planning/scripts/moveit_ik_demo.py
2，打印位姿
rqt_plot /joint_states/position[0]:position[1]:position[2]:position[3]:position[4]:position[5]:position[6]




