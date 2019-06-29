## DWAPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
- `costmap_ros_->getRobotPose`函数得到当前的位姿，调用`costmap_2d_ros.cpp`中对应的函数

- `planner_util_.getLocalPlan`函数裁剪全局路径，调用`local_planner_util.cpp`中对应的函数，在base_local_planner包中

- `dp_->updatePlanAndLocalCosts`传入上面2个函数参数和`costmap_ros_->getRobotFootprint()`函数中返回的参数（也是`costmap_2d_ros.cpp`中对应的函数）。dp_中设置关于cost的一些参数

- `latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_)`函数来判断到达目标点后的停止旋转，调用base_local_planner包中`latched_stop_rotate_controller.cpp`中的