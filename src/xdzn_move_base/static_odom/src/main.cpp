#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include "../include/Robot.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry");

    ros::NodeHandle n;

    My_Robot mr(n);

    ros::Subscriber cmd_sub = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&My_Robot::MoveRobot, &mr, _1)); //("cmd_vel", 100, mr.MoveRobot);

    ros::Rate r(30.0);
    while ( n.ok() ) {
        ros::spinOnce();
        mr.pubOdom();
        mr.pubPose();
        r.sleep();
    }

    ros::spin();
    return 0;
}