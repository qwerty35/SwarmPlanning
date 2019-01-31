#include <fstream>
#include <iostream>
#include <math.h>

#include <timer.hpp>

#include <ECBSLauncher.hpp>
#include <BoxGenerator.hpp>
#include <SwarmPlanner.hpp>

//Third Party
#include <ecbs.hpp>
#include <environment.hpp>
#include <timer.hpp>

//ROS
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>

//Octomap
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

using libMultiRobotPlanning::ECBS;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

bool has_octomap = false;
bool has_path = false;

int x_min, y_min, z_min, x_max, y_max, z_max;
std::shared_ptr<octomap::OcTree> octree_obj;

void octomapCallback(const octomap_msgs::Octomap octomap_msg)
{
    if(has_octomap)
        return;

    octree_obj.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(octomap_msg)));

    has_octomap = true;
}

void visObsbox(ros::NodeHandle n, std::vector<ros::Publisher>& obsbox_vis_pubs, const std::vector<std::vector<std::pair<octomap::point3d,octomap::point3d>>>& obstacle_boxes){
    obsbox_vis_pubs.resize(obstacle_boxes.size());
    for (int a=0; a< obstacle_boxes.size(); a++){
        obsbox_vis_pubs.at(a) = n.advertise<visualization_msgs::MarkerArray>("/mav" + std::to_string(a) + "/obstacle_box", 1);

        visualization_msgs::MarkerArray mk_array;
        visualization_msgs::Marker mk;
        mk.header.frame_id = "world";
        mk.header.stamp = ros::Time::now();
        mk.ns = "obsbox";
        mk.type = visualization_msgs::Marker::CUBE;
        mk.action = visualization_msgs::Marker::ADD;

        mk.pose.orientation.x = 0.0;
        mk.pose.orientation.y = 0.0;
        mk.pose.orientation.z = 0.0;
        mk.pose.orientation.w = 1.0;

        mk.color.a = 0.2;
        mk.color.r = 0.0;
        mk.color.g = 1.0;
        mk.color.b = 0.0;

        for (int i=0; i<obstacle_boxes.at(a).size(); i++){
            mk.id = i;
            std::pair<octomap::point3d, octomap::point3d> obstacle_box = obstacle_boxes.at(a).at(i);
            mk.pose.position.x = (obstacle_box.first.x()+obstacle_box.second.x())/2.0;
            mk.pose.position.y = (obstacle_box.first.y()+obstacle_box.second.y())/2.0;
            mk.pose.position.z = (obstacle_box.first.z()+obstacle_box.second.z())/2.0;

            mk.scale.x = obstacle_box.second.x()-obstacle_box.first.x();
            mk.scale.y = obstacle_box.second.y()-obstacle_box.first.y();
            mk.scale.z = obstacle_box.second.z()-obstacle_box.first.z();

            mk_array.markers.emplace_back(mk);
        }
        obsbox_vis_pubs.at(a).publish(mk_array);
    }
}

void publishTrajCoef(ros::NodeHandle n, std::vector<ros::Publisher>& traj_coef_pubs, const std::vector<std_msgs::Float64MultiArray>& msgs_traj_coef){
    int qn = msgs_traj_coef.size();
    traj_coef_pubs.resize(qn);
    for(int qi = 0; qi < qn; qi++){
        traj_coef_pubs[qi] = n.advertise<std_msgs::Float64MultiArray>("/mav" + std::to_string(qi) + "/traj_coef", 1);
        traj_coef_pubs[qi].publish(msgs_traj_coef[qi]);
    }
}

int main(int argc, char* argv[]) {
    ros::init (argc, argv, "init_traj_generator");
    ros::NodeHandle n( "~" );
    ros::V_string args;
    ros::removeROSArgs(argc, argv, args);

    ROS_WARN("Init_traj_generator");
    ros::Subscriber octomap_sub = n.subscribe( "/octomap_full", 1, octomapCallback );
//    ros::Publisher init_poses_pub = n.advertise<geometry_msgs::PoseArray>("/world/init_poses", 1);
//    ros::Publisher ecbs_path_pub = n.advertise<nav_msgs::Path>("/ecbs/init_traj", 1);
//    ros::Publisher obsbox_pub = n.advertise<nav_msgs::Path>("/box/obstacle_box", 1);
    std::vector<ros::Publisher> obsbox_vis_pubs;
    ros::Publisher traj_info_pub = n.advertise<std_msgs::Float64MultiArray>("/traj_info", 1);
    std::vector<ros::Publisher> traj_coef_pubs;

    double x_size, y_size, z_size;
    double ecbs_w, ecbs_xy_res, ecbs_z_res, ecbs_margin;
    double box_xy_res, box_z_res, box_margin;
    double plan_time_step, plan_downwash;
    int plan_Npoly, plan_n;

    n.param<double>("world/x_size", x_size, 12);
    n.param<double>("world/y_size", y_size, 12);
    n.param<double>("world/z_size", z_size, 2);

    n.param<double>("ecbs/w", ecbs_w, 1.3);
    n.param<double>("ecbs/xy_res", ecbs_xy_res, 1);
    n.param<double>("ecbs/z_res", ecbs_z_res, 1.5);
    n.param<double>("ecbs/margin", ecbs_margin, 0.3);

    n.param<double>("box/xy_res", box_xy_res, 0.1);
    n.param<double>("box/z_res", box_z_res, 0.1);
    n.param<double>("box/margin", box_margin, 0.4);

    n.param<double>("plan/time_step", plan_time_step, 1);
    n.param<double>("plan/downwash", plan_downwash, 2.5);
    n.param<int>("plan/Npoly", plan_Npoly, 5);
    n.param<int>("plan/n", plan_n, 3);


    x_min = floor(-x_size/2);
    y_min = floor(-y_size/2);
    z_min = 0;
    x_max = ceil(x_size/2);
    y_max = ceil(y_size/2);
    z_max = ceil(z_size);

    geometry_msgs::PoseArray init_poses;
    std::vector<octomap::point3d> start, goal;
    std::vector<double> quad_size;
    int input_size = 7;
    if (args.size() == 1 || args.size() % input_size != 1){
        ROS_WARN("Usage: init_traj_publisher <x_start> <y_start> <z_start> <x_goal> <y_goal> <z_goal> <quad_size> ... \n");
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

            start.emplace_back(octomap::point3d(std::stof(args.at(i+1)), std::stof(args.at(i+2)), std::stof(args.at(i+3))));
            goal.emplace_back(octomap::point3d(std::stof(args.at(i+4)), std::stof(args.at(i+5)), std::stof(args.at(i+6))));
            quad_size.emplace_back(std::stof(args.at(i+7)));
        }
    }

    ros::Rate rate(20);
    std::shared_ptr<ECBSLauncher> ecbs_launcher;
    std::shared_ptr<BoxGenerator> box_generator;
    std::shared_ptr<SwarmPlanner> swarm_planner;
    while (ros::ok()) {
        if (has_octomap && !has_path) {
            Timer timer;

            timer.reset();
            ROS_INFO("Launch ECBS");
            ecbs_launcher.reset(new ECBSLauncher(x_min, y_min, z_min, x_max, y_max, z_max,
                                                 ecbs_w, ecbs_xy_res, ecbs_z_res, ecbs_margin,
                                                 start, goal, octree_obj));
            ecbs_launcher.get()->update();

            ROS_INFO("Generate obstacle box");
            box_generator.reset(new BoxGenerator(x_min, y_min, z_min, x_max, y_max, z_max,
                                                 box_xy_res, box_z_res, box_margin,
                                                 ecbs_xy_res, ecbs_z_res, ecbs_launcher.get()->makespan,
                                                 ecbs_launcher.get()->ecbs_traj, plan_time_step, octree_obj));
            box_generator.get()->update();

            ROS_INFO("Optimize swarm trajectory");
            swarm_planner.reset(new SwarmPlanner(start, goal, quad_size, plan_Npoly, plan_n, plan_downwash,
                                                 box_generator.get()->obstacle_boxes,
                                                 box_generator.get()->relative_boxes,
                                                 box_generator.get()->ts_each, box_generator.get()->ts_total));
            swarm_planner.get()->update();

            timer.stop();
            std::cout << "Overall runtime: " << timer.elapsedSeconds() << std::endl;

            has_path = true;
        }
        if (has_path) {
//            init_poses_pub.publish(init_poses);
//            ecbs_path_pub.publish(ecbs_launcher.get()->msgs_ecbs_traj);
//            obsbox_pub.publish(box_generator.get()->msgs_obstacle_boxes);
            visObsbox(n, obsbox_vis_pubs, box_generator.get()->obstacle_boxes);
            traj_info_pub.publish(swarm_planner.get()->msgs_traj_info);
            publishTrajCoef(n, traj_coef_pubs, swarm_planner.get()->msgs_traj_coef);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
