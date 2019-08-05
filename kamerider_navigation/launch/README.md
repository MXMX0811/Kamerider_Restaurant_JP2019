# kamerider_navigation
对于Turtlebot地盘，导航以及路径规划的实习

#从action生成msg文件的方法
rosrun actionlib_msgs genaction.py -o msg/ action/<action_file>

<<<<<<< HEAD
#建图方法以及说明
1.修改准则：
  建图线速度不要超过0.2m/s，角速度不要超过1rad/s！！！！

2.建图要求：
  1）保证上面的速度要求。！！！！
  2）首先控制机器人沿着场地的轮廓线走若干圈，同时通过rviz观察建图效果，保证没有畸变。
  3）保证机器人均速运动，遇到转弯处务必转直角。！！！！
  4）从一个房间到另一个房间一定要注意房间的轮廓线是否畸变。
  5）如此当轮廓确定准确后，控制机器人建立房间内部的细节地图。
  6）一定要多次建图！！！！

3.使用
  roslaunch kamerider_navigation map_setup.launch
  roslaunch turtlebot_teleop keyboard_teleop.launch

4.保存地图
  rosrun map_server map_saver -f /home/kamerider/catkin_ws/src/kamerider_navigation/maps/<name>
=======
|      |                                                              |
| ---- | ------------------------------------------------------------ |
|      |                                                              |
|      | 1.建立地图：                                                 |
|      | 1）在navigation_test中的launch文件夹下，找到map_setup.launch文件，新建终端输入： |
|      | roslaunch navigation_test map_setup.launch                   |
|      |                                                              |
|      | 2）建图完毕后需要保存地图，新建终端键入：                    |
|      | rosrun map_server map_saver -f /home/isi/2017_ws/src/navigation_test/maps/<name> |
|      |                                                              |
|      | 3）这样就将建立好的地图保存在navigation_test中的maps文件夹中了。 |
|      |                                                              |
|      | 4）关闭所有终端。                                            |
|      | 注意：参见map_setup说明文档。（launch文件夹下）              |
|      |                                                              |
|      | 2.采集waypoints:                                             |
|      | 1)在navigation_test中的launch文件下，找到get_waypoints.launch，新建终端输入： |
|      | roslaunch navigation_test get_waypoints.launch               |
|      | 2)通过tele_op(键盘)控制机器人到所要求的位置，在小窗口输入“get” |
|      | 3)采集完所有点后输入“stop”完成所有采集工作                   |
|      |                                                              |
|      | 3.启动navigation_test                                        |
|      | 1）在navigation_test中的launch文件下，找到navi_setup.launch，新建终端输入： |
|      | roslaunch navigation_test demo.launch                        |
|      |                                                              |
|      | 2）在rviz中设定机器人在地图上的初始坐标：                    |
|      | 点击rviz上的Estimate Pose，在地图上估计位置，设定机器人的位置和朝向,并摆正地图，否则导航误差大 |
|      |                                                              |
|      | 3）开始navigation。                                          |
>>>>>>> 1f5e05ce09436f9f53813b10de954fdee32bc4d8
