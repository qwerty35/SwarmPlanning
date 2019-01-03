#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <math.h>
#include <random>

int main (int argc, char** argv) {
    ros::init (argc, argv, "init_cond_publisher");
    ros::NodeHandle n( "~" );

    ROS_WARN("Init_cond_publisher");
    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);

    ros::Publisher init_poses_pub = n.advertise<geometry_msgs::PoseArray>("/world/init_poses", 1);
    ros::Publisher boundary_pub = n.advertise<geometry_msgs::Pose>("/world/boundary",1);

    float x_size, y_size, z_size;
    n.param<float>("world/x_size", x_size, 12);
    n.param<float>("world/y_size", y_size, 12);
    n.param<float>("world/z_size", z_size, 2);

    geometry_msgs::PoseArray init_poses;
    geometry_msgs::Pose boundary;

    int input_size = 7;
    if (args.size() == 1 || args.size() % input_size != 1){
        ROS_WARN("Usage: init_cond_publisher <x_start> <y_start> <z_start> <x_goal> <y_goal> <z_goal> <quad_size> ... \n");
        return -1;
    }
    else{
        for (int i = 0; i < args.size()-1; i = i + input_size){
            geometry_msgs::Pose init_pose;
            init_pose.position.x = std::stof(args.at(i+1));
            init_pose.position.y = std::stof(args.at(i+2));
            init_pose.position.z = std::stof(args.at(i+3));
            init_pose.orientation.x = std::stof(args.at(i+4));
            init_pose.orientation.y = std::stof(args.at(i+5));
            init_pose.orientation.z = std::stof(args.at(i+6));
            init_pose.orientation.w = std::stof(args.at(i+7));

            init_poses.poses.push_back(init_pose);
        }
    }

    boundary.position.x = -x_size/2;
    boundary.position.y = -y_size/2;
    boundary.position.z = 0;

    boundary.orientation.x = x_size/2;
    boundary.orientation.y = y_size/2;
    boundary.orientation.z = z_size;


    ros::Rate rate(10);
    while (ros::ok()){
        init_poses_pub.publish(init_poses);
        boundary_pub.publish(boundary);
        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}

