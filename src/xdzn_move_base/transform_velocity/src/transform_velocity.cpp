#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <sstream>
#include <math.h>    


ros::Publisher write_pub;
ros::Subscriber cmd_sub;

double wheel_interval;//轮间距
double wheel_radius;//轮半径
int nn; //电机转1s的编码器数

double vL,vR;  //速度：左轮，右轮

void cmd_vel_CB(const geometry_msgs::Twist& cmd_vel){

    ROS_INFO("wheel_interval, wheel_radius, nn");
    ROS_INFO("[%f, %f, %d]",wheel_interval,wheel_radius,nn);

    // ROS_INFO("Received a /cmd_vel message!");
    // ROS_INFO("Linear Components:[%f, %f, %f]", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.linear.z);
    // ROS_INFO("Angular Components:[%f, %f, %f]", cmd_vel.angular.x, cmd_vel.angular.y, cmd_vel.angular.z);
    double Vc = cmd_vel.linear.x;
    double Wc = cmd_vel.angular.z;
    
    //根据线速度和角速度计算左右轮速度
    vL = Vc - wheel_interval*Wc/2;
    vR = Vc + wheel_interval*Wc/2;
    // ROS_INFO("vL,vR:[%f ,%f]",vL, vR);
    
    //转换为编码器数值
    int num_Left = (vL*nn) /(2*M_PI*wheel_radius);
    int num_Right = (vR*nn) /(2*M_PI*wheel_radius);
    // ROS_INFO("Left,Right:[%d, %d]",num_Left, num_Right);

    std_msgs::String msg;
    std::stringstream ss;
    ss << "1@" << num_Left << "@" << num_Right;
    msg.data = ss.str();
    ROS_INFO("%s",msg.data.c_str());

    write_pub.publish(msg);


}

int main(int argc, char** argv){
    
    ros::init(argc, argv, "transform_velocity");
    ros::NodeHandle n;
    
    n.param<double>("transform_velocity/wheel_interval", wheel_interval, 0.3);
    n.param<double>("transform_velocity/wheel_radius", wheel_radius, 0.1);
    n.param<int>("transform_velocity/nn", nn, 8000);

    ROS_INFO("[%f, %f, %d]", wheel_interval, wheel_radius, nn);
    
    write_pub = n.advertise<std_msgs::String>("xdzn_serial/write", 1000);
    cmd_sub = n.subscribe("cmd_vel", 1000, cmd_vel_CB);
    
    ros::spin();

}