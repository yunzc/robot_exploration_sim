#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include "geom.h"

class Environment{
public:
    Environment(); // constructor 
    std::vector<Point> pts; // store the points in the environment 
    Pose pursuer_pose; 
    bool map_read;
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void pursuerCallback(const visualization_msgs::Marker::ConstPtr& msg);
};

// constructor
Environment::Environment(void) {
   map_read = false; 
}

void Environment::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    std_msgs::Header header = msg->header; 
    nav_msgs::MapMetaData info = msg->info; 
    if (!map_read){
        ROS_INFO("Got map %d %d", info.width, info.height); 
        for (unsigned int x = 0; x < info.width; x++){
            for (unsigned int y = 0; y < info.height; y++){
                if (msg->data[x + info.width*y] < 50){
                    Point pt; 
                    pt.x = x*info.resolution; pt.y = y*info.resolution;
                    pt.clean = false; 
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

    // get occupancy grid 
    ros::Subscriber map_sub = nh.subscribe("map", 10, &Environment::mapCallback, &env);

    // subscrbe to pursuer location
    ros::Sub

    while (nh.ok()){
        visualization_msgs::Marker points; 
        // fill in the basic marker infos 
        points.header.frame_id = "/map"; 
        points.header.stamp = ros::Time::now(); 
        points.ns = "evader_location";
        points.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0; // unimportant just filling in
        points.id = 0; points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.05; points.scale.y = 0.05; // size of points 
        points.color.r = 1.0f; points.color.a = 1.0; // non transparent red points 

        // start storing position 
        int count = 0; 
        for (int i = 0; i < env.pts.size(); i++){
            if (!env.pts[i].clean){
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
        // ROS_INFO("%d out of %d dirty points remaining...", count, (int)env.pts.size()); 
        marker_pub.publish(points);
        ros::spinOnce();
        r.sleep();
    }
}