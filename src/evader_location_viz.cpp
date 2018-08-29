#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <math.h> 
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

    double fov = 80.0/180.0*M_PI; 

    // get occupancy grid 
    ros::Subscriber map_sub = nh.subscribe("map", 10, &Environment::mapCallback, &env);

    // subscrbe to pursuer location
    ros::Subscriber purs_sub = nh.subscribe("pursuer_pose", 10, &Environment::pursuerCallback, &env); 

    while (nh.ok()){
        //// first get the triangular region of the pursuer fov 
        Point triang[3]; 
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
        rotation(x1, y1, yaw); 
        rotation(x2, y2, yaw); 
        triang[0] = purs; 
        triang[1].x = purs.x + x1; triang[1].y = purs.y + y1; 
        triang[2].x = purs.x + x2; triang[2].y = purs.y + y2; 

        // first clean the in views 
        for (int i = 0; i < env.pts.size(); i++){
            // clean if in view 
            if (isInside(triang, 3, env.pts[i])){
                // env.pts[i].clean = true; 
            }
        }

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
            if (isInside(triang, 3, env.pts[i])){
                // if inside is now clean and don't show 
                env.pts[i].clean = true; 
            }else if (!env.pts[i].clean){
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