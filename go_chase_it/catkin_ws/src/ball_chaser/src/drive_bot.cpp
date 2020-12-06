#include<ros/ros.h>
#include"geometry_msgs/Twist.h"
#include<ball_chaser/DriveToTarget.h>

class drive_bot
{
private:
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_publisher;
    ros::ServiceServer drive_service;

public:
    drive_bot();
    ~drive_bot();
    bool drive_service_cb(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res);
};

drive_bot::drive_bot()
{
    cmd_vel_publisher = nh.advertise<geometry_msgs::Twist>("/my_robot/cmd_vel", 10);
    drive_service = nh.advertiseService("/ball_chaser/command_robot", &drive_bot::drive_service_cb, this);
}

drive_bot::~drive_bot()
{
}

bool drive_bot::drive_service_cb(ball_chaser::DriveToTarget::Request &req,
                                 ball_chaser::DriveToTarget::Response &res)
{
    ROS_INFO("Drive service called with %f linear velocity and %f angular velocity.",req.linear_x, req.angular_z);
    geometry_msgs::Twist cmd;
    cmd.linear.x = req.linear_x;
    cmd.angular.z = req.angular_z;
    cmd_vel_publisher.publish(cmd);
    res.msg_feedback = std::to_string(req.linear_x) + "," + std::to_string(req.angular_z);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drive_bot_node");
    drive_bot obj_drive_bot;
    ros::spin();
    return 0;
}