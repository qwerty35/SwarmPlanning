//
// Created by jungwon on 19. 1. 21.
//

#pragma once

#include <fstream>
#include <iostream>

#include <timer.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <math.h>

class BoxGenerator {
public:
    nav_msgs::Path msgs_obstacle_boxes;
    std::vector<std::vector<std::pair<octomap::point3d,octomap::point3d>>> obstacle_boxes;
    std::vector<std::vector<std::vector<std::pair<int, double>>>> relative_boxes;
    std::vector<std::vector<double>> ts_each;
    std::vector<double> ts_total;

    BoxGenerator(double _x_min, double _y_min, double _z_min,
                 double _x_max, double _y_max, double _z_max,
                 double _box_xy_res, double _box_z_res, double _box_margin,
                 double _ecbs_xy_res, double _ecbs_z_res, int _ecbs_makespan,
                 const std::vector<std::vector<octomap::point3d>>& _ecbs_traj,
                 double _plan_time_step,
                 std::shared_ptr<octomap::OcTree> _octree_obj)
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
               ecbs_traj(_ecbs_traj),
               plan_time_step(_plan_time_step)
    {
        double maxDist = 1;
        octomap::point3d min_point3d(x_min, y_min, z_min);
        octomap::point3d max_point3d(x_max, y_max, z_max);
        distmap_obj.reset(new DynamicEDTOctomap(maxDist, _octree_obj.get(), min_point3d, max_point3d, false));
        distmap_obj.get()->update();

        total_time = (_ecbs_makespan+2+2) * plan_time_step; // +2 : start, goal point +2 : matlab error???????????????????????????
        qn = ecbs_traj.size();
    }

    void update(){
        updateObsBox();
        updateRelBox();
        updateTs();
    }

private:
    double x_min,y_min,z_min,x_max,y_max,z_max;
    double box_xy_res, box_z_res, box_margin, ecbs_xy_res, ecbs_z_res;
    std::vector<std::vector<octomap::point3d>> ecbs_traj;
    std::shared_ptr<DynamicEDTOctomap> distmap_obj;
    std::vector<int> ts_total_int;
    double plan_time_step, total_time;
    int qn;

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


    void updateObsBox(){
        octomap::point3d min_prev_pt(0,0,0);
        octomap::point3d max_prev_pt(0,0,0);
        double x_prev, y_prev, z_prev, dx, dy, dz;
        double num_err = 0.0001;

        Timer timer;
        timer.reset();

        obstacle_boxes.resize(qn);
        ts_each.resize(qn);
        for (size_t qi = 0; qi < qn; ++qi) {
            for (int i = 0; i < ecbs_traj[qi].size(); i++) {
                auto state = ecbs_traj[qi][i];
                double x = state.x();
                double y = state.y();
                double z = state.z();

                // Skip when current point is in prev box
                if (isPointInBox(octomap::point3d(x, y, z), min_prev_pt, max_prev_pt)) {
                    continue;
                }

                // Initialize box
                octomap::point3d min_box_pt(x-box_xy_res/2, y-box_xy_res/2, z-box_z_res/2);
                octomap::point3d max_box_pt(x+box_xy_res/2, y+box_xy_res/2, z+box_z_res/2);

                // Select expansion direction to guarantee box connection between adjacent points
                if (i > 0) {
                    auto state_prev = ecbs_traj[qi][i-1];
                    x_prev = state_prev.x();
                    y_prev = state_prev.y();
                    z_prev = state_prev.z();

                    dx = round((x_prev-x)/ecbs_xy_res);
                    dy = round((y_prev-y)/ecbs_xy_res);
                    dz = round((z_prev-z)/ecbs_z_res);

                    int axis_pref = ((dx > 0) - (dx < 0)) * 1 + ((dy > 0) - (dy < 0)) * 2 + ((dz > 0) - (dz < 0)) * 3;
                    assert(dx > -2 && dx < 2 && dy > -2 && dy < 2 && dz > -2 && dz < 2);

                    expand_box(min_box_pt, max_box_pt, box_margin, axis_pref);

                    assert(isPointInBox(octomap::point3d(x_prev, y_prev, z_prev), min_box_pt, max_box_pt));
                } else {
                    expand_box(min_box_pt, max_box_pt, box_margin, 0);
                }
                //assert(isPointInBox(octomap::point3d(x,y,z),min_box_pt,max_box_pt));

                geometry_msgs::PoseStamped msgs_obstacle_box;
                msgs_obstacle_box.header.seq = qi;
                msgs_obstacle_box.pose.position.x = min_box_pt.x();
                msgs_obstacle_box.pose.position.y = min_box_pt.y();
                msgs_obstacle_box.pose.position.z = min_box_pt.z();
                msgs_obstacle_box.pose.orientation.x = max_box_pt.x();
                msgs_obstacle_box.pose.orientation.y = max_box_pt.y();
                msgs_obstacle_box.pose.orientation.z = max_box_pt.z();
                msgs_obstacle_boxes.poses.push_back(msgs_obstacle_box);

                obstacle_boxes[qi].emplace_back(std::make_pair(min_box_pt, max_box_pt));

                min_prev_pt = min_box_pt;
                max_prev_pt = max_box_pt;
            }

            // Generate box time segment
            int box_max = obstacle_boxes[qi].size();
            int path_max = ecbs_traj[qi].size();
            Eigen::MatrixXd box_log = Eigen::MatrixXd::Zero(box_max, path_max);
            std::vector<int> ts;

            for(int i=0; i<box_max; i++){
                for(int j=0; j<path_max; j++){
                    if(isPointInBox(ecbs_traj[qi][j], obstacle_boxes[qi][i].first, obstacle_boxes[qi][i].second)){
                        if (j==0){
                            box_log(i,j) = 1;
                        }
                        else{
                            box_log(i,j) = box_log(i,j-1) + 1;
                        }
                    }
                }
            }

            int box_iter = box_max-1;
            int path_iter = path_max-1;
            while(box_iter >= 0 && path_iter >= 0){
                path_iter = path_iter - box_log(box_iter, path_iter) + 1;
                box_iter = box_iter - 1;

                if(box_iter >= 0){
                    if(box_log(box_iter, path_iter) == 0){
                        std::cerr << "invalid box connection!" << std::endl;
                    }
                    int count=1;
                    while(path_iter+count < path_max && box_log(box_iter, path_iter+count) > 0){
                        count++;
                    }
                    ts.emplace_back(floor(path_iter+count/2.0)+1);////////////////////////////////////////////////////////////???????????????????
                }
            }

            // update ts_each
            ts_each[qi].emplace_back(0);
            for(int i=ts.size()-1; i>=0; i--){
                ts_each[qi].emplace_back(ts[i] * plan_time_step);
            }
            ts_each[qi].emplace_back(total_time);

            ts_total_int.insert(ts_total_int.end(), ts.begin(), ts.end());
        }

        timer.stop();
        std::cout << "ObsBoxing success!" << std::endl;
        std::cout << "ObsBoxing runtime: " << timer.elapsedSeconds() << std::endl;
    }


    void updateRelBox(){
        int sector_range[6] = {-3,-2,-1,1,2,3};
        double num_err = 0.0001;

        Timer timer;
        timer.reset();

        relative_boxes.resize(qn);
        for(int qi=0; qi<qn; qi++){
            relative_boxes[qi].resize(qn);
            for(int qj=qi+1; qj<qn; qj++){
                int path_max = std::max<int>(ecbs_traj[qi].size(), ecbs_traj[qj].size());
                int path_min = std::min<int>(ecbs_traj[qi].size(), ecbs_traj[qj].size());
                Eigen::MatrixXd sector_log = Eigen::MatrixXd::Zero(6, path_max);

                for(int iter=0; iter<path_max; iter++){
                    // Get rel_pose
                    int rel_pose[4];
                    double dx, dy, dz;

                    if(iter < path_min){
                        dx = round((ecbs_traj[qj][iter].x()-ecbs_traj[qi][iter].x())/ecbs_xy_res);
                        dy = round((ecbs_traj[qj][iter].y()-ecbs_traj[qi][iter].y())/ecbs_xy_res);
                        dz = round((ecbs_traj[qj][iter].z()-ecbs_traj[qi][iter].z())/ecbs_z_res);
                    }
                    else if(ecbs_traj[qi].size() == path_min){
                        dx = round((ecbs_traj[qj][iter].x()-ecbs_traj[qi][path_min-1].x())/ecbs_xy_res);
                        dy = round((ecbs_traj[qj][iter].y()-ecbs_traj[qi][path_min-1].y())/ecbs_xy_res);
                        dz = round((ecbs_traj[qj][iter].z()-ecbs_traj[qi][path_min-1].z())/ecbs_z_res);
                    }
                    else{
                        dx = round((ecbs_traj[qj][path_min-1].x()-ecbs_traj[qi][iter].x())/ecbs_xy_res);
                        dy = round((ecbs_traj[qj][path_min-1].y()-ecbs_traj[qi][iter].y())/ecbs_xy_res);
                        dz = round((ecbs_traj[qj][path_min-1].z()-ecbs_traj[qi][iter].z())/ecbs_z_res);
                    }
                    rel_pose[1] = (dx>num_err)-(dx<-num_err); //Caution: (q1_size+q2_size)/grid_size should be small enough!
                    rel_pose[2] = (dy>num_err)-(dy<-num_err);
                    rel_pose[3] = (dz>num_err)-(dz<-num_err);

                    // Save sector information
                    for(int i=0; i<6; i++){
                        int sector = sector_range[i];
                        int sgn = (i>2)-(i<3);
                        if(rel_pose[abs(sector)]*sgn > 0){
                            if(iter == 0){
                                sector_log(i,iter) = 1;
                            }
                            else{
                                sector_log(i,iter) = sector_log(i,iter-1)+1;
                            }
                        }
                    }
                }

//                std::cout << sector_log << std::endl;


                //find minimum jump sector path (heuristic greedy search)
                int iter = path_max-1;
                int sector_next = -1;
                int count_next = sector_log.col(iter).maxCoeff(&sector_next);

                relative_boxes[qi][qj].emplace_back(std::make_pair(sector_range[sector_next], total_time));
                iter = iter - count_next + 1;

                while(iter > 1){
                    int sector_curr;
                    int count;

                    // if there is no intersection then allow to jump sector
                    // except jumping through quadrotor (i.e. +x -> -x jumping is not allowed)
                    if(sector_log.col(iter).maxCoeff(&sector_curr) <= 1){
                        iter = iter - 1;
                        int sector_opp = 6-1-sector_next;

                        if (sector_log.col(iter).maxCoeff(&sector_curr) <= 0){
                            std::cerr << "Invalid Path, missing link" << std::endl;
                        }
                        else if(sector_curr == sector_opp){
                            bool flag = false;
                            for(int i=0; i<6; i++){
                                if(i != sector_opp && sector_log(i,iter) == sector_log.col(iter).maxCoeff(&sector_curr)){
                                    flag = true;
                                    break;
                                }
                            }
                            if(!flag) {
                                std::cerr << "Invalid Path, jumping through quadrotor" << std::endl;
                            }
                        }
                        count = 0;
                    }
                    else{
                        count = 1;
                        while(sector_log(sector_curr,iter+count) > 0){
                            count++;
                        }

                    }

                    if(count == 0){
                        relative_boxes[qi][qj].emplace_back(std::make_pair(sector_range[sector_curr], (iter+1)*plan_time_step));/////////////////////////
                        ts_total_int.emplace_back(iter+1);///////////////////////////////////////////////////////////////////////////////////////////////
                    }
                    else{
                        relative_boxes[qi][qj].emplace_back(std::make_pair(sector_range[sector_curr], (floor(iter+count/2.0)+1)*plan_time_step));////////
                        ts_total_int.emplace_back(floor(iter+count/2.0)+1);//////////////////////////////////////////////////////////////////////////////
                    }
                    sector_next = sector_curr;
                    iter = iter - sector_log.col(iter).maxCoeff() + 1;
                }
            }
        }

        timer.stop();
        std::cout << "RelBoxing success!" << std::endl;
        std::cout << "RelBoxing runtime: " << timer.elapsedSeconds() << std::endl;
    }

    void updateTs(){
        Timer timer;
        timer.reset();

        std::sort(ts_total_int.begin(), ts_total_int.end());
        ts_total_int.erase(std::unique(ts_total_int.begin(), ts_total_int.end()), ts_total_int.end());

        // update ts_total
        ts_total.emplace_back(0);
        for(int i = 0; i<ts_total_int.size(); i++){
            ts_total.emplace_back(ts_total_int[i] * plan_time_step);
        }
        ts_total.emplace_back(total_time);

        timer.stop();
        std::cout << "Time segment updated!" << std::endl;
        std::cout << "Time segment runtime: " << timer.elapsedSeconds() << std::endl;
    }
};


