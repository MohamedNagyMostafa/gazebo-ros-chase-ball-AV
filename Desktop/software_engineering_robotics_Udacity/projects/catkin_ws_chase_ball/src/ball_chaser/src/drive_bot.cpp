#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

ros::Publisher motorCommandPublisher;

bool handleDriveRequestCallback(ball_chaser::DriveToTarget::Request& request,
                                ball_chaser::DriveToTarget::Response& response)
{

    geometry_msgs::Twist velocity;

    velocity.linear.x     = request.linear_x;
    velocity.angular.z    = request.angular_z;

    motorCommandPublisher.publish(velocity);

    ros::Duration(2).sleep();

    response.msg_feedback   = "The robot moves with a linear velocity: "+  std::to_string(request.linear_x) +" and an angular velocity:" + std::to_string(request.angular_z);

    return true;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "drive_bot");

    ros::NodeHandle nodeHandle;

    motorCommandPublisher   = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::ServiceServer driveRobotService    = nodeHandle.advertiseService("/ball_chaser/command_robot", handleDriveRequestCallback);

    ROS_INFO("Ready to send robot commands");

    ros::spin();

    return 0;
}