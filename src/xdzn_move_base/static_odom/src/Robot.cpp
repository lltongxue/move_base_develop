#include "../include/Robot.h"

My_Robot::My_Robot(ros::NodeHandle _nh)
{
    _n = _nh;
    odom_pub = _n.advertise<nav_msgs::Odometry>("odom", 50);
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    move = false;
    x = 0.0;
    y = 0.0;
    th = 0.0;

    vx = 0.0;
    vy = -0.0;
    vth = 0.0;
}

void My_Robot::start(){
    // while ( _n.ok() && !move ) {
    //     pubOdom(0,0,0,0,0,0);
    // }
}

void My_Robot::pubOdom()
{
    current_time = ros::Time::now();    
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    // ROS_INFO_STREAM("publish odom tf: " << vx);
    //publish the message
    odom_pub.publish(odom);
    last_time = current_time;
}

void My_Robot::pubPose()
{
    current_time = ros::Time::now();
    geometry_msgs::Quaternion pos_quat = tf::createQuaternionMsgFromYaw(0);
    geometry_msgs::TransformStamped pos_trans;
    pos_trans.header.stamp = current_time;
    pos_trans.header.frame_id = "map";
    pos_trans.child_frame_id = "odom";

    pos_trans.transform.translation.x = 0;
    pos_trans.transform.translation.y = 0;
    pos_trans.transform.translation.z = 0.0;
    pos_trans.transform.rotation = pos_quat;

    // ROS_INFO_STREAM("publish pose tf: " << pos_trans.transform.translation.x);
    //send the transform
    robot_broadcaster.sendTransform(pos_trans);
}

void My_Robot::MoveRobot(const geometry_msgs::Twist::ConstPtr &msg)
{
    vx = msg->linear.x;
    vy = msg->linear.y;
    vth = msg->angular.z;
}