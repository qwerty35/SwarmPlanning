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

#include <math.h>

bool has_map = false;
bool has_path = false;
bool has_box = false;

double x_min,y_min,z_min,x_max,y_max,z_max;
double xy_res, z_res, margin;
std::shared_ptr<octomap::OcTree> octree_obj;
//
std::shared_ptr<DynamicEDTOctomap> distmap_obj;
//
nav_msgs::Path ecbs_path_msg;

void octomapCallback(const octomap_msgs::Octomap _ecbs_octomap_msg)
{
    octree_obj.reset(dynamic_cast<octomap::OcTree*>(octomap_msgs::fullMsgToMap(_ecbs_octomap_msg)));
    //
    double maxDist = 1;
    octomap::point3d min_point3d(x_min, y_min, z_min);
    octomap::point3d max_point3d(x_max, y_max, z_max);
    distmap_obj.reset(new DynamicEDTOctomap(maxDist, octree_obj.get(), min_point3d, max_point3d, false));
    distmap_obj.get()->update();
    //

    has_map = true;
}

void pathCallback(const nav_msgs::Path _ecbs_path_msg)
{
    ecbs_path_msg = _ecbs_path_msg;
    has_path = true;
}

bool isObstacleInBox(const octomap::point3d& min_box_pt, const octomap::point3d& max_box_pt){
    double numel_e = 0.0001;
    for(double i = min_box_pt.x(); i < max_box_pt.x() + numel_e; i += xy_res) {
        for(double j = min_box_pt.y(); j < max_box_pt.y() + numel_e; j += xy_res) {
            for(double k = min_box_pt.z(); k < max_box_pt.z() + numel_e; k += z_res) {
                octomap::point3d cur_point(i,j,k);
                float dist = distmap_obj.get()->getDistance(cur_point);

                if(dist < margin){
                    return true;
                }
            }
        }
    }

    return false;
}

//bool isObstacleInBox(const octomap::point3d& min_box_pt, const octomap::point3d& max_box_pt){
//    for (octomap::OcTree::leaf_bbx_iterator it = octree_obj->begin_leafs_bbx(min_box_pt, max_box_pt),
//                 end = octree_obj->end_leafs_bbx(); it != end; ++it) {
//        octomap::OcTreeNode *node = octree_obj->search(it.getCoordinate());
//        if (node && octree_obj->isNodeOccupied(node)) {
//            return true;
//        }
//    }
//    return false;
//}

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
                    min_cand_pt(axis) = min_cand_pt(axis) - z_res;
                }
                else {
                    min_cand_pt(axis) = min_cand_pt(axis) - xy_res;
                }
                min_update_pt(axis) = min_cand_pt(axis);
            }
            else{
                min_update_pt(axis-3) = max_cand_pt(axis-3);
                if (axis == 5){
                    max_cand_pt(axis-3) = max_cand_pt(axis-3) + z_res;
                }
                else{
                    max_cand_pt(axis-3) = max_cand_pt(axis-3) + xy_res;
                }
                max_update_pt(axis-3) = max_cand_pt(axis-3);
            }
        }
    }

//    for (int axis = 0; axis < 3; axis++) {
//        min_box_pt(axis) = min_box_pt(axis) + margin;
//        max_box_pt(axis) = max_box_pt(axis) - margin;
//    }


}

int main(int argc, char* argv[]) {
    ros::init (argc, argv, "obstacle_box_generator");
    ros::NodeHandle n( "~" );

    ROS_WARN("Obstacle box generator");
    ros::Subscriber octomap_sub = n.subscribe( "/octomap_full", 5, octomapCallback );
    ros::Subscriber ecbs_path_sub = n.subscribe( "/ecbs/init_traj", 5, pathCallback );

    ros::Publisher obsbox_pub = n.advertise<nav_msgs::Path>("/box/obstacle_box", 5);

    double x_size, y_size, z_size, ecbs_xy_res, ecbs_z_res;

    n.param<double>("world/x_size", x_size, 12);
    n.param<double>("world/y_size", y_size, 12);
    n.param<double>("world/z_size", z_size, 2);

    n.param<double>("ecbs/xy_res", ecbs_xy_res, 1);
    n.param<double>("ecbs/z_res", ecbs_z_res, 1.5);

    n.param<double>("box/xy_res", xy_res, 0.1);
    n.param<double>("box/z_res", z_res, 0.1);
    n.param<double>("box/margin", margin, 0.4);

    x_min = floor(-x_size/2);
    y_min = floor(-y_size/2);
    z_min = 0;
    x_max = ceil(x_size/2);
    y_max = ceil(y_size/2);
    z_max = ceil(z_size);

    nav_msgs::Path obstacle_boxes;
    while (ros::ok()) {
        if (has_map && has_path && !has_box) {
            nav_msgs::Path _obstacle_boxes;

            octomap::point3d min_prev_pt(0,0,0);
            octomap::point3d max_prev_pt(0,0,0);
            double a_prev = -1;
            double x_prev, y_prev, z_prev, dx, dy, dz;

            Timer timer;
            timer.reset();
            for (int i = 0; i < ecbs_path_msg.poses.size(); i++) {
                double a = ecbs_path_msg.poses.at(i).header.seq;
                double x = ecbs_path_msg.poses.at(i).pose.position.x;
                double y = ecbs_path_msg.poses.at(i).pose.position.y;
                double z = ecbs_path_msg.poses.at(i).pose.position.z;

                // skip when current point is in prev box
                if (a == a_prev && isPointInBox(octomap::point3d(x,y,z),min_prev_pt,max_prev_pt)){
                    continue;
                }

                octomap::point3d min_box_pt(x - xy_res / 2, y - xy_res / 2, z - z_res / 2);
                octomap::point3d max_box_pt(x + xy_res / 2, y + xy_res / 2, z + z_res / 2);

                if(i > 0 && a == a_prev) {
                    x_prev = ecbs_path_msg.poses.at(i-1).pose.position.x;
                    y_prev = ecbs_path_msg.poses.at(i-1).pose.position.y;
                    z_prev = ecbs_path_msg.poses.at(i-1).pose.position.z;

                    dx = round((x_prev-x)/ecbs_xy_res); //guarantee connection between adjacent points
                    dy = round((y_prev-y)/ecbs_xy_res);
                    dz = round((z_prev-z)/ecbs_z_res);

                    int axis_pref = ((dx>0)-(dx<0))*1 + ((dy>0)-(dy<0))*2 + ((dz>0)-(dz<0))*3;
                    assert(dx>-2 && dx<2 && dy>-2 && dy<2 && dz>-2 && dz<2);

                    expand_box(min_box_pt, max_box_pt, margin, axis_pref);

                    assert(isPointInBox(octomap::point3d(x_prev,y_prev,z_prev),min_box_pt,max_box_pt));
                }
                else{
                    expand_box(min_box_pt, max_box_pt, margin, 0);
                }
//                assert(isPointInBox(octomap::point3d(x,y,z),min_box_pt,max_box_pt));


                geometry_msgs::PoseStamped obstacle_box;
                obstacle_box.header.seq = a;

                obstacle_box.pose.position.x = min_box_pt.x();
                obstacle_box.pose.position.y = min_box_pt.y();
                obstacle_box.pose.position.z = min_box_pt.z();

                obstacle_box.pose.orientation.x = max_box_pt.x();
                obstacle_box.pose.orientation.y = max_box_pt.y();
                obstacle_box.pose.orientation.z = max_box_pt.z();

                _obstacle_boxes.poses.push_back(obstacle_box);

                min_prev_pt = min_box_pt;
                max_prev_pt = max_box_pt;
                a_prev = a;
            }
            timer.stop();
            obstacle_boxes = _obstacle_boxes;

            ROS_WARN("Boxing successful! \n");
            std::cout << "Boxing runtime: " << timer.elapsedSeconds() << std::endl;

            has_map = false;
            has_path = false;
            has_box = true;
        }
        if (has_box){
            obsbox_pub.publish(obstacle_boxes);
        }
        ros::spinOnce();
    }

    return 0;
}
