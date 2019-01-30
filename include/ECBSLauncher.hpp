//
// Created by jungwon on 19. 1. 21.
//

#pragma once

#include <fstream>
#include <iostream>

#include <timer.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Header.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <math.h>

#include <environment.hpp>

using libMultiRobotPlanning::ECBS;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;

class ECBSLauncher {
public:
    std::vector<std::vector<octomap::point3d>> ecbs_traj;
    nav_msgs::Path msgs_ecbs_traj;
    int makespan;

    ECBSLauncher(int _x_min, int _y_min, int _z_min,
                 int _x_max, int _y_max, int _z_max, /////////////////////////////////////int?????
                 double _ecbs_w, double _ecbs_xy_res, double _ecbs_z_res, double _ecbs_margin,
                 std::vector<octomap::point3d> _start,
                 std::vector<octomap::point3d> _goal,
                 std::shared_ptr<octomap::OcTree> _octree_obj)
             : x_min(_x_min),
               y_min(_y_min),
               z_min(_z_min),
               x_max(_x_max),
               y_max(_y_max),
               z_max(_z_max),
               ecbs_w(_ecbs_w),
               ecbs_xy_res(_ecbs_xy_res),
               ecbs_z_res(_ecbs_z_res),
               ecbs_margin(_ecbs_margin),
               start(_start),
               goal(_goal),
               octree_obj(_octree_obj)
    {
        dimx = (_x_max - _x_min) / ecbs_xy_res + 1;
        dimy = (_y_max - _y_min) / ecbs_xy_res + 1;
        dimz = (_z_max - _z_min) / ecbs_z_res + 1;

        setObstacles();
        setWaypoints();
    }

    void update(){
        Environment mapf(dimx, dimy, dimz, obstacles, goalLocations);
        ECBS<State, Action, int, Conflict, Constraints, Environment> ecbs(mapf, ecbs_w);
        std::vector<PlanResult<State, Action, int> > solution;

        Timer timer;
        bool success = ecbs.search(startStates, solution);
        timer.stop();

        if (success) {
            ROS_WARN("Planning successful! \n");
            int cost = 0;
            makespan = 0;
            for (const auto &s : solution) {
                cost += s.cost;
                makespan = std::max<int>(makespan, s.cost);
            }

            std::cout << "statistics:" << std::endl;
            std::cout << "  cost: " << cost << std::endl;
            std::cout << "  makespan: " << makespan << std::endl;
            std::cout << "  runtime: " << timer.elapsedSeconds() << std::endl;
            std::cout << "  highLevelExpanded: " << mapf.highLevelExpanded() << std::endl;
            std::cout << "  lowLevelExpanded: " << mapf.lowLevelExpanded() << std::endl;

            ecbs_traj.resize(solution.size());
            for (size_t a = 0; a < solution.size(); ++a) {
                geometry_msgs::PoseStamped sol;
                sol.header.seq = a;
                sol.pose.position.x = start.at(a).x();
                sol.pose.position.y = start.at(a).y();
                sol.pose.position.z = start.at(a).z();
                msgs_ecbs_traj.poses.push_back(sol);

                ecbs_traj.at(a).emplace_back(octomap::point3d(start.at(a).x(), start.at(a).y(), start.at(a).z()));

                for (const auto &state : solution[a].states) {
                    sol.pose.position.x = state.first.x * ecbs_xy_res + x_min;
                    sol.pose.position.y = state.first.y * ecbs_xy_res + y_min;
                    sol.pose.position.z = state.first.z * ecbs_z_res + z_min;
                    msgs_ecbs_traj.poses.push_back(sol);

                    ecbs_traj.at(a).emplace_back(octomap::point3d(state.first.x * ecbs_xy_res + x_min,
                                                                  state.first.y * ecbs_xy_res + y_min,
                                                                  state.first.z * ecbs_z_res + z_min));
                }

                sol.pose.position.x = goal.at(a).x();
                sol.pose.position.y = goal.at(a).y();
                sol.pose.position.z = goal.at(a).z();
                msgs_ecbs_traj.poses.push_back(sol);

                ecbs_traj.at(a).emplace_back(octomap::point3d(goal.at(a).x(), goal.at(a).y(), goal.at(a).z()));
            }
        }
        else {
            ROS_ERROR("Planning Failed!\n");
        }
    }

private:
    double x_min,y_min,z_min,x_max,y_max,z_max;
    int dimx, dimy, dimz;
    double ecbs_w, ecbs_xy_res, ecbs_z_res, ecbs_margin;
    std::shared_ptr<octomap::OcTree> octree_obj;
    std::unordered_set<Location> obstacles;
    std::vector<octomap::point3d> start, goal;
    std::vector<State> startStates;
    std::vector<Location> goalLocations;

    void setObstacles(){
        // create Euclidean Distance map
        double maxDist = 1;
        octomap::point3d min_point3d(x_min, y_min, z_min);
        octomap::point3d max_point3d(x_max, y_max, z_max);
        DynamicEDTOctomap distmap(maxDist, octree_obj.get(),
                                  min_point3d,
                                  max_point3d,
                                  false);
        distmap.update();

        // Get obstacle
        int x,y,z;
        for (double k = z_min; k <= z_max; k += ecbs_z_res ){
            for (double i = x_min; i <= x_max; i += ecbs_xy_res ){
                for (double j = y_min; j <= y_max; j += ecbs_xy_res ){
                    octomap::point3d cur_point(i,j,k);
                    float dist = distmap.getDistance(cur_point);

                    assert(dist>=0);

                    if (dist < ecbs_margin){
                        x = round((i-x_min)/ecbs_xy_res);
                        y = round((j-y_min)/ecbs_xy_res);
                        z = round((k-z_min)/ecbs_z_res);
                        obstacles.insert(Location(x, y, z));
                    }
                }
            }
        }
    }

    void setWaypoints(){
        double xig,yig,zig,xfg,yfg,zfg;
        for(int i=0; i<start.size(); i++){
            xig = round((start.at(i).x()-x_min)/ecbs_xy_res);
            yig = round((start.at(i).y()-y_min)/ecbs_xy_res);
            zig = round((start.at(i).z()-z_min)/ecbs_z_res);
            xfg = round((goal.at(i).x()-x_min)/ecbs_xy_res);
            yfg = round((goal.at(i).y()-y_min)/ecbs_xy_res);
            zfg = round((goal.at(i).z()-z_min)/ecbs_z_res);

            assert(obstacles.find(Location(xig,yig,zig)) == obstacles.end());
            assert(obstacles.find(Location(xfg,yfg,zfg)) == obstacles.end());

            startStates.emplace_back(State(0, xig, yig, zig));
            goalLocations.emplace_back(Location(xfg,yfg,zfg));
        }
    }
};


