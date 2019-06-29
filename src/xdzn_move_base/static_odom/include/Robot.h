#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

class My_Robot
{

public:
    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformBroadcaster robot_broadcaster;
    ros::NodeHandle _n;

    ros::Time current_time, last_time;
    bool move;
    double x;
    double y;
    double th;

    double vx;
    double vy;
    double vth;

    My_Robot(ros::NodeHandle _n);

    void pubOdom();

    void pubPose();

    void MoveRobot(const geometry_msgs::Twist::ConstPtr &msg);

    void start();
};