#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geom.h"

class Environment{
public:
    Environment(); // constructor 
    std::vector<Point> pts; // store the points in the environment 
    bool map_read;
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
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

int main(int argc, char** argv){
    ros::init(argc, argv, "evader_location");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("target_points", 10); 

    ros::Rate r(30);

    Environment env; 

    // get occupancy grid 
    ros::Subscriber map_sub = nh.subscribe("map", 10, &Environment::mapCallback, &env);

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
        ROS_INFO("%d out of %d dirty points remaining...", count, (int)env.pts.size()); 
        marker_pub.publish(points);
        ros::spinOnce();
        r.sleep();
    }
}
// int main( int argc, char** argv){
//   ros::init(argc, argv, "evader_points");
//   ros::NodeHandle n;
//   ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

//   ros::Rate r(30);

//   srand(time(NULL));

//   while (ros::ok()){
//     visualization_msgs::Marker points;
//     points.header.frame_id = "/map";
//     points.header.stamp = ros::Time::now();
//     points.ns = "evader_points";
//     points.action = visualization_msgs::Marker::ADD;
//     points.pose.orientation.w = 1.0;

//     points.id = 0;

//     points.type = visualization_msgs::Marker::POINTS;

//     // POINTS markers use x and y scale for width/height respectively
//     points.scale.x = 0.05;
//     points.scale.y = 0.05;

//     // Points are green
//     points.color.r = 1.0f;
//     points.color.a = 1.0;

//     // Create the vertices for the points and lines
//     for (uint32_t i = 0; i < 5-00; ++i){
//       float x = (double)rand()/RAND_MAX*4;
//       float y = (double)rand()/RAND_MAX*3;

//       geometry_msgs::Point p;
//       p.x = x;
//       p.y = y;
//       p.z = 0;

//       points.points.push_back(p);
//     }


//     marker_pub.publish(points);

//     r.sleep();

//   }
// }