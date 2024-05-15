#include <ros/ros.h>
#include <potbot_lib/DiffDriveController.h>
#include <potbot_msgs/ObstacleArray.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <PF_nav/PF_navConfig.h>
#include <geometry_msgs/PoseArray.h>
#include <random>
#include <visualization_msgs/MarkerArray.h>
#include <potbot_lib/Utility.h>
#include <cmath>
#include <vector>


std::vector<geometry_msgs::Pose> marker_positions;


void Marker_callback(const visualization_msgs::MarkerArray& marker_array)
{   
    marker_positions.clear();

	for(const auto& marker : marker_array.markers)
    {
        geometry_msgs::Pose marker_pose = marker.pose;

        marker_positions.push_back(marker_pose);

        ROS_INFO("Marker ID: %d, Position: [x: %f, y: %f, z: %f]", 
                  marker.id, marker_pose.position.x, marker_pose.position.y, marker_pose.position.z);
    }
}

void RobotPose_Callback(const nav_msgs::Odometry& odom_pose)
{
    //nav_msgs::Odometry odom_pose ;
    
    double x = odom_pose.pose.pose.position.x;
    double y = odom_pose.pose.pose.position.y;
    double z = odom_pose.pose.pose.position.z;

    double yaw = potbot_lib::utility::get_Yaw(odom_pose.pose.pose.orientation);
    
    ROS_INFO("Position: x=%.2f, y=%.2f, z=%.2f", x, y, z);
    ROS_INFO("Orientation (RPY): yaw=%.2f", yaw);

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_Visualization");
    ros::NodeHandle nh;

    // サブスクライバの作成
    ros::Subscriber sub_marker = nh.subscribe("marker", 1000, Marker_callback);
    ros::Subscriber sub_robot_pose = nh.subscribe("robot_pose", 1000, RobotPose_Callback);

    ros::spin();

    return 0;
}