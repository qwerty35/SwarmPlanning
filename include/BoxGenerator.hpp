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

class BoxGenerator {
public:
    nav_msgs::Path msgs_obstacle_boxes;
    std::vector<std::vector<std::pair<octomap::point3d,octomap::point3d>>> obstacle_boxes;

    BoxGenerator(double _x_min, double _y_min, double _z_min,
                 double _x_max, double _y_max, double _z_max,
                 double _box_xy_res, double _box_z_res, double _box_margin,
                 double _ecbs_xy_res, double _ecbs_z_res,
                 std::shared_ptr<octomap::OcTree> _octree_obj,
                 const std::vector<std::vector<octomap::point3d>>& _solution)
             : x_min(_x_min),
               y_min(_y_min),
               z_min(_z_min),
               x_max(_x_max),
               y_max(_y_max),
               z_max(_z_max),
               box_xy_res(_box_xy_res),
               box_z_res(_box_z_res),
               box_margin(_box_margin),
               ecbs_xy_res(_ecbs_xy_res),
               ecbs_z_res(_ecbs_z_res),
               solution(_solution)
    {
        double maxDist = 1;
        octomap::point3d min_point3d(x_min, y_min, z_min);
        octomap::point3d max_point3d(x_max, y_max, z_max);
        distmap_obj.reset(new DynamicEDTOctomap(maxDist, _octree_obj.get(), min_point3d, max_point3d, false));
        distmap_obj.get()->update();
    }

    void update(){
        octomap::point3d min_prev_pt(0,0,0);
        octomap::point3d max_prev_pt(0,0,0);
        double x_prev, y_prev, z_prev, dx, dy, dz;

        Timer timer;
        timer.reset();
        obstacle_boxes.resize(solution.size());
        for (size_t a = 0; a < solution.size(); ++a) {
            for (int i = 0; i < solution[a].size(); i++) {
                auto state = solution[a].at(i);
                double x = state.x();
                double y = state.y();
                double z = state.z();

                // skip when current point is in prev box
                if (isPointInBox(octomap::point3d(x, y, z), min_prev_pt, max_prev_pt)) {
                    continue;
                }

                octomap::point3d min_box_pt(x-box_xy_res/2, y-box_xy_res/2, z-box_z_res/2);
                octomap::point3d max_box_pt(x+box_xy_res/2, y+box_xy_res/2, z+box_z_res/2);

                if (i > 0) {
                    auto state_prev = solution[a].at(i-1);
                    x_prev = state_prev.x();
                    y_prev = state_prev.y();
                    z_prev = state_prev.z();

                    dx = round((x_prev-x)/ecbs_xy_res); //guarantee connection between adjacent points
                    dy = round((y_prev-y)/ecbs_xy_res);
                    dz = round((z_prev-z)/ecbs_z_res);

                    int axis_pref = ((dx > 0) - (dx < 0)) * 1 + ((dy > 0) - (dy < 0)) * 2 + ((dz > 0) - (dz < 0)) * 3;
                    assert(dx > -2 && dx < 2 && dy > -2 && dy < 2 && dz > -2 && dz < 2);

                    expand_box(min_box_pt, max_box_pt, box_margin, axis_pref);

                    assert(isPointInBox(octomap::point3d(x_prev, y_prev, z_prev), min_box_pt, max_box_pt));
                } else {
                    expand_box(min_box_pt, max_box_pt, box_margin, 0);
                }
//                assert(isPointInBox(octomap::point3d(x,y,z),min_box_pt,max_box_pt));

                geometry_msgs::PoseStamped msgs_obstacle_box;
                msgs_obstacle_box.header.seq = a;

                msgs_obstacle_box.pose.position.x = min_box_pt.x();
                msgs_obstacle_box.pose.position.y = min_box_pt.y();
                msgs_obstacle_box.pose.position.z = min_box_pt.z();

                msgs_obstacle_box.pose.orientation.x = max_box_pt.x();
                msgs_obstacle_box.pose.orientation.y = max_box_pt.y();
                msgs_obstacle_box.pose.orientation.z = max_box_pt.z();

                msgs_obstacle_boxes.poses.push_back(msgs_obstacle_box);

                obstacle_boxes.at(a).emplace_back(std::make_pair(min_box_pt, max_box_pt));

                min_prev_pt = min_box_pt;
                max_prev_pt = max_box_pt;
            }
        }
        timer.stop();

        std::cout << "Boxing successful!" << std::endl;
        std::cout << "Boxing runtime: " << timer.elapsedSeconds() << std::endl;
    }

private:
    double x_min,y_min,z_min,x_max,y_max,z_max;
    double box_xy_res, box_z_res, box_margin, ecbs_xy_res, ecbs_z_res;
    std::shared_ptr<DynamicEDTOctomap> distmap_obj;
    std::vector<std::vector<octomap::point3d>> solution;

    bool isObstacleInBox(const octomap::point3d& min_box_pt, const octomap::point3d& max_box_pt){
        double numel_e = 0.0001;
        for(double i = min_box_pt.x(); i < max_box_pt.x() + numel_e; i += box_xy_res) {
            for(double j = min_box_pt.y(); j < max_box_pt.y() + numel_e; j += box_xy_res) {
                for(double k = min_box_pt.z(); k < max_box_pt.z() + numel_e; k += box_z_res) {
                    octomap::point3d cur_point(i,j,k);
                    float dist = distmap_obj.get()->getDistance(cur_point);
                    assert(dist>=0);
                    if(dist < box_margin){
                        return true;
                    }
                }
            }
        }

        return false;
    }

    bool isBoxInBoundary(const octomap::point3d& min_box_pt, const octomap::point3d& max_box_pt){
        if(min_box_pt.x() >= x_min && min_box_pt.y() >= y_min && min_box_pt.z() >= z_min &&
           max_box_pt.x() <= x_max && max_box_pt.y() <= y_max && max_box_pt.z() <= z_max){
            return true;
        }

        return false;
    }

    bool isPointInBox(const octomap::point3d& point, const octomap::point3d& min_box_pt, const octomap::point3d& max_box_pt){
        if(point.x() >= min_box_pt.x() && point.y() >= min_box_pt.y() && point.z() >= min_box_pt.z() &&
           point.x() <= max_box_pt.x() && point.y() <= max_box_pt.y() && point.z() <= max_box_pt.z()){
            return true;
        }

        return false;
    }

    void expand_box(octomap::point3d& min_box_pt, octomap::point3d& max_box_pt, double margin, int axis_pref) {
        octomap::point3d min_cand_pt, max_cand_pt, min_update_pt, max_update_pt;
        int axis;
        int axis_cand[6] = {0,1,2,3,4,5};

        if (axis_pref < 0){
            axis_pref = -axis_pref-1;
        }
        else if (axis_pref > 0){
            axis_pref = axis_pref+2;
        }
        axis_cand[0] = axis_pref;
        axis_cand[axis_pref] = 0;

        for (int i = 0; i < 6; i++) {
            axis = axis_cand[i];

            //Box expansion
            min_cand_pt = min_box_pt;
            max_cand_pt = max_box_pt;
            min_update_pt = min_box_pt;
            max_update_pt = max_box_pt;

            //check update_box only! update_box < cand_box
            while (!isObstacleInBox(min_update_pt, max_update_pt) && isBoxInBoundary(min_update_pt, max_update_pt)) {
                min_box_pt = min_cand_pt;
                max_box_pt = max_cand_pt;

                if (axis < 3) {
                    max_update_pt(axis) = min_cand_pt(axis);
                    if (axis == 2){
                        min_cand_pt(axis) = min_cand_pt(axis) - box_z_res;
                    }
                    else {
                        min_cand_pt(axis) = min_cand_pt(axis) - box_xy_res;
                    }
                    min_update_pt(axis) = min_cand_pt(axis);
                }
                else{
                    min_update_pt(axis-3) = max_cand_pt(axis-3);
                    if (axis == 5){
                        max_cand_pt(axis-3) = max_cand_pt(axis-3) + box_z_res;
                    }
                    else{
                        max_cand_pt(axis-3) = max_cand_pt(axis-3) + box_xy_res;
                    }
                    max_update_pt(axis-3) = max_cand_pt(axis-3);
                }
            }
        }

        //for (int axis = 0; axis < 3; axis++) {
            //min_box_pt(axis) = min_box_pt(axis) + margin;
            //max_box_pt(axis) = max_box_pt(axis) - margin;
        //}

    }
};


