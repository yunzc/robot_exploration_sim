#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

// this node would subscribe to the trajectory loader 
// then move the pursuer accordingly 
class Pursuer{
public:
    geometry_msgs::PoseArray trajectory; 
    void trajCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
};

void Pursuer::trajCallback(const geometry_msgs::PoseArray::ConstPtr& msg){
	trajectory = *msg;
	ROS_INFO("trajectory received..."); 
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pursuer_location");
    ros::NodeHandle nh;
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("pursuer_pose", 10); 

    ros::Rate r(10);

    Pursuer p; 

    // get occupancy grid 
    ros::Subscriber traj_sub = nh.subscribe("trajectory", 10, &Pursuer::trajCallback, &p);
    ros::spinOnce();

    while (p.trajectory.poses.size() == 0){
    	ROS_INFO("getting trajectory...");
    	ros::spinOnce();
    	r.sleep();
    } 


    int i = 0; 
    while (nh.ok() && i < p.trajectory.poses.size()){
        visualization_msgs::Marker pursuer_pose; 
        // fill in the basic marker infos 
        pursuer_pose.header.frame_id = "/map"; 
        pursuer_pose.header.stamp = ros::Time::now(); 
        pursuer_pose.ns = "pursuer_location";
        pursuer_pose.pose = p.trajectory.poses[i]; 
        std::cout << p.trajectory.poses[i]; 
        pursuer_pose.id = 0; pursuer_pose.type = visualization_msgs::Marker::ARROW;
        pursuer_pose.scale.x = 0.15; pursuer_pose.scale.y = 0.05; 
        pursuer_pose.scale.z = 0.05; // size of arrow
        pursuer_pose.color.b = 1.0f; pursuer_pose.color.a = 1.0; // non transparent blue
        marker_pub.publish(pursuer_pose);
        ros::spinOnce(); 
        r.sleep();
        i++; 
    }

}