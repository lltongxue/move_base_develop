基于ROS move_base的导航控制模块
====================

基于里程计信息及机器人位姿信息的导航控制模块

## 部署方法
首先卸载安装ros时自动安装的导航包相关模块，然后在部署源码包

在ROS工作空间中执行
```sh
git clone http://192.168.1.100:8888/ROS/xdzn_move_base.git
cd xdzn_move_base
git checkout develop
cp move_base/config/tpl/* move_base/config/
cp move_base/rviz/tpl/* move_base/rviz/
```

在工作空间根目录执行 `catkin_make` 进行编译

完成后，运行 `roslaunch move_base robot_nav.launch`