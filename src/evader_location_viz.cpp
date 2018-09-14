#include <cmath>
#include <math.h> 
#include <iostream>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include "geom.h"

#define _USE_MATH_DEFINES
#define infy 1000

class Environment{
public:
    Environment(); // constructor 
    std::vector<Point> pts; // store the points in the environment 
    geometry_msgs::Pose pursuer_pose; 
    bool map_read;
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void pursuerCallback(const visualization_msgs::Marker::ConstPtr& msg);
    std::vector<Point> vertices; // vertices of the environment 
    int map_height; 
    int map_width; 
};

// constructor
Environment::Environment(void) {
   map_read = false; 
   //// read from file to get coordinates 
   std::ifstream inFile; 
   inFile.open("/home/yun/vis-pe/env/env1.txt");
    if (!inFile.is_open()){
        std::cout << "error opening input file" << std::endl; 
    }
    // first get first line for number of vertices 
    std::string line; 
    std::getline(inFile, line); 
    std::string str1; 
    int num; 
    std::stringstream ss(line); 
    ss >> str1 >> num; 
    // read through the points and store 
    while (std::getline(inFile, line)){
        std::stringstream ss(line); 
        double double1; 
        double double2;
        ss >> double1 >> double2;
        Point pt; pt.x = double1; pt.y = double2;
        vertices.push_back(pt); 
    }
}

void Environment::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    std_msgs::Header header = msg->header; 
    nav_msgs::MapMetaData info = msg->info; 
    if (!map_read){
        ROS_INFO("Got map %d %d", info.width, info.height); 
        map_height = info.height; 
        map_width = info.width; 
        for (unsigned int x = 0; x < info.width; x++){
            for (unsigned int y = 0; y < info.height; y++){
                if (msg->data[x + info.width*y] < 50){
                    // in environment 
                    Point pt; 
                    pt.x = x*info.resolution; pt.y = y*info.resolution;
                    pt.clean = false; 
                    pt.in = true; 
                    pts.push_back(pt);
                }else{
                    Point pt; 
                    pt.x = x*info.resolution; pt.y = y*info.resolution;
                    pt.clean = true; 
                    pt.in = false; 
                    pts.push_back(pt);
                }   
            }
        }
    }
    map_read = true; 
}

void Environment::pursuerCallback(const visualization_msgs::Marker::ConstPtr& msg){
    pursuer_pose = msg->pose; 
}

int main(int argc, char** argv){
    ros::init(argc, argv, "evader_location");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("target_points", 10); 

    ros::Rate r(30);

    Environment env; 

    double slack = 2.0; // some slack for fov to make sure it cleans everything in view
    double fov = (80.0 + slack)/180.0*M_PI; 

    // get occupancy grid 
    ros::Subscriber map_sub = nh.subscribe("map", 10, &Environment::mapCallback, &env);

    // subscrbe to pursuer location
    ros::Subscriber purs_sub = nh.subscribe("pursuer_pose", 10, &Environment::pursuerCallback, &env); 

    while (nh.ok()){
        //// first get the triangular region of the pursuer fov 
        // triang with "4" points (trapezoid) to make is no point is missed 
        Point triang[4]; 
        // one point of triangle is where the pursuer is currently at 
        Point purs;
        purs.x = env.pursuer_pose.position.x; purs.y = env.pursuer_pose.position.y;
        geometry_msgs::Quaternion quat = env.pursuer_pose.orientation;
        tf::Quaternion q(quat.x, quat.y, quat.z, quat.w); 
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        // get other two points with fov and heading (yaw)
        double x1 = infy; double y1 = infy*tan(fov/2); 
        double x2 = infy; double y2 = -infy*tan(fov/2); 
        double x3 = 0; double y3 = 0.05; 
        double x4 = 0; double y4 = -0.05;
        rotation(x1, y1, yaw); 
        rotation(x2, y2, yaw); 
        rotation(x3, y3, yaw);
        rotation(x4, y4, yaw);
        triang[0].x = purs.x + x3; triang[0].y = purs.y + y3;  
        triang[1].x = purs.x + x1; triang[1].y = purs.y + y1; 
        triang[2].x = purs.x + x2; triang[2].y = purs.y + y2; 
        triang[3].x = purs.x + x4; triang[3].y = purs.y + y4;

        visualization_msgs::Marker points; 
        // fill in the basic marker infos 
        points.header.frame_id = "/map"; 
        points.header.stamp = ros::Time::now(); 
        points.ns = "evader_location";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0; // unimportant just filling in
        points.id = 0; points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.01; points.scale.y = 0.01; points.scale.z = 0.01; // size of points 
        points.color.r = 1.0f; points.color.a = 1.0; // non transparent red points 

        // start storing position 
        int count = 0; 
        for (int i = 0; i < env.pts.size(); i++){
            if (env.pts[i].in){
                if (isInside(triang, 4, env.pts[i])){
                    // if inside check if in view
                    bool blocked = false; 
                    // ray cast check 
                    for (int j = 0; j < env.vertices.size(); j++){
                        int next = j + 1; 
                        if (next == env.vertices.size()){
                            next = 0; 
                        }
                        if (doIntersect(purs, env.pts[i], env.vertices[j], env.vertices[next])
                            && !onSegment(env.vertices[j], purs, env.vertices[next])){ 
                            blocked = true;
                            break; 
                        }
                    }
                    if (!blocked){
                        // clean if in view  and not blocked 
                        env.pts[i].clean = true;  
                    }
                }else{
                    // if not in view check if recontaminated 
                    if (env.pts[i].clean){
                        int jumps[4] = {-1, -env.map_height, 1, env.map_height};
                        for (int j = 0; j < sizeof(jumps)/sizeof(jumps[0]); j++){
                            if (i + jumps[j] > 0 && i + jumps[j] < env.pts.size()){
                                if (!env.pts[i + jumps[j]].clean && env.pts[i + jumps[j]].in
                                    && (i + jumps[j])%env.map_height != 0
                                    && (i + jumps[j])%env.map_height != -1%env.map_height){
                                    env.pts[i].clean = false; 
                                    break; 
                                }
                            }
                        }
                    }
                }
            }
            if (!env.pts[i].clean and env.pts[i].in){
                // not clean: the evader might be at this point
                float x = env.pts[i].x;
                float y = env.pts[i].y; 

                geometry_msgs::Point p;
                p.x = x;
                p.y = y;
                p.z = 0;

                // push into array 
                points.points.push_back(p);
                count += 1; 
            }
        }
        ROS_INFO("%d out of %d dirty points remaining...", count, (int)env.pts.size()); 
        marker_pub.publish(points);
        ros::spinOnce();
        r.sleep();
    }
}