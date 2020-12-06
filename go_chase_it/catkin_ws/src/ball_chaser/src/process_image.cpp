#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <ball_chaser/DriveToTarget.h>

class process_image
{
private:
    ros::ServiceClient drive_client;
    ros::Subscriber image_subscriber;
    ros::NodeHandle nh;
    // What percentage of image from center should the ball be tracked.
    int ball_in_image_center;

public:
    process_image(int argc, char **argv);
    ~process_image();
    void image_subscriber_callback(const sensor_msgs::Image img);
    void drive_robot(float linear_x, float angular_z);
};

process_image::process_image(int argc, char **argv)
{
    ros::service::waitForService("/ball_chaser/command_robot");
    drive_client = nh.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
    image_subscriber = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_raw", 2, &process_image::image_subscriber_callback, this);
    if (argc == 2)
    {
        ball_in_image_center = atoi(argv[1]);
        ROS_INFO("Received : %d", ball_in_image_center);
        if (ball_in_image_center < 1 || ball_in_image_center > 100)
            ball_in_image_center = 10;
    }
    else
    {
        ROS_ERROR("Please pass proper arguments.");
        ROS_ERROR("Usage :");
        ROS_ERROR("rosrun ball_chaser process_image <ball_in_image_center>");
        ros::shutdown();
    }
    
}

process_image::~process_image()
{
}

void process_image::drive_robot(float linear_x, float angular_z)
{
    ball_chaser::DriveToTarget srv_data;
    srv_data.request.linear_x = linear_x;
    srv_data.request.angular_z = angular_z;
    if (drive_client.call(srv_data))
    {
        ROS_INFO("Robot moved with velocity %f in linear and %f in angular direction.", linear_x, angular_z);
    }
    else
    {
        ROS_INFO("Failed to call service.");
    }
}

void process_image::image_subscriber_callback(const sensor_msgs::Image img_msg)
{
    int ball_color = 255;
    int ball_column = -1;
    int center1, center2;
    center1 = img_msg.width / 2 - img_msg.width * ball_in_image_center / 200;
    center2 = img_msg.width / 2 + img_msg.width * ball_in_image_center / 200;
    for (int i = 0; i < img_msg.height; i++)
    {
        for (int j = 0; j < img_msg.step; j=j+3)
        {
            // Check all three channels RGB for white color.
            if (img_msg.data[i * img_msg.step + j] == ball_color && img_msg.data[i * img_msg.step + j + 1] == ball_color && img_msg.data[i * img_msg.step + j + 2] == ball_color)
            {
                ball_column = j / (img_msg.step / img_msg.width);
                break;
            }
        }
        if (ball_column != -1)
            break;
    }
    if (0 <= ball_column && ball_column < center1)
    {
        drive_robot(0, 0.5);
    }
    else if (center1 <= ball_column && ball_column < center2)
    {
        drive_robot(0.5, 0);
    }
    else if (center2 <= ball_column && ball_column < img_msg.width)
    {
        drive_robot(0, -0.5);
    }
    else
    {
        drive_robot(0, 0);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "process_image_node");
    process_image obj(argc, argv);
    ros::spin();
    return 0;
}
