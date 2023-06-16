//
// Created by nagy on 6/15/23.
//
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient driveRobotClient;

void driveRobot(float linearX, float angularZ)
{
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x    = linearX;
    srv.request.angular_z   = angularZ;

    if(!driveRobotClient.call(srv))
        ROS_ERROR("Failed to call service drive_robot");
}

void processImageCallback(const sensor_msgs::Image image)
{
    int whitePixel  = 255;

    int numberOfChannels            = image.step/image.width;
    int leftMovementTolerance       = image.width/3;
    int forwardMovementTolerance    = leftMovementTolerance + image.width/3;

    std::vector<int8_t> directionDecision = {0, 0, 0};
    int maxDirectionPro = -1;


    for(int i =0; i < image.height * image.width * numberOfChannels; i+=3) {
        int redChannel = image.data[i];
        int greenChannel = image.data[i + 1];
        int blueChannel = image.data[i + 2];

        bool isWhitePixel = (redChannel == whitePixel && greenChannel == whitePixel && blueChannel == whitePixel);

        if (isWhitePixel) {

            int imageSegmentLocation = (i % (image.width * numberOfChannels)) / numberOfChannels;
            if (imageSegmentLocation <= leftMovementTolerance) {
                directionDecision[0]++;
                maxDirectionPro = (directionDecision[0] > maxDirectionPro) ? 0 : maxDirectionPro;
            } else if (imageSegmentLocation <= forwardMovementTolerance) {
                directionDecision[1]++;
                maxDirectionPro = (directionDecision[1] > maxDirectionPro) ? 1 : maxDirectionPro;
            } else {
                directionDecision[2]++;
                maxDirectionPro = (directionDecision[2] > maxDirectionPro) ? 2 : maxDirectionPro;
            }
        }
    }
    switch(maxDirectionPro)
    {
        case 0:
            driveRobot(0, 0.1);
            ROS_INFO("Robot is turning left");
            break;
        case 1:
            driveRobot(0.3, 0);
            ROS_INFO("Robot is moving forward");
            break;
        case 2:
            driveRobot(0.0, -.1);
            ROS_INFO("Robot is turning right");
            break;
        default:
            driveRobot(0.0, 0.0);
            ROS_INFO("Robot is stopped");
    }

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "process_image");
    ros::NodeHandle nodeHandle;

    driveRobotClient = nodeHandle.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    ros::Subscriber cameraSubscriber    = nodeHandle.subscribe("/camera/rgb/image_raw", 10, processImageCallback);

    ros::spin();

    return 0;

}

