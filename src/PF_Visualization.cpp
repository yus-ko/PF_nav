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
#include <math.h>

std::vector<geometry_msgs::Pose> marker_positions;

double robot_pose_x = 0.0, robot_pose_y = 0.0, robot_pose_z = 0.0, robot_pose_yaw = 0.0;
double radius = 3.0, start_angle = 0.0, end_angle = 0.0, angle_area = 85.2 * M_PI / 180;


void Marker_callback(const visualization_msgs::MarkerArray& marker_array)
{   
    marker_positions.clear();

	for(const auto& marker : marker_array.markers)
    {
        geometry_msgs::Pose marker_pose = marker.pose;

        marker_positions.push_back(marker_pose);

        // ROS_INFO("Marker ID: %d, Position: [x: %f, y: %f, z: %f]", 
        //           marker.id, marker_pose.position.x, marker_pose.position.y, marker_pose.position.z);
    }
}

void RobotPose_Callback(const nav_msgs::Odometry& odom_pose)
{
    
     robot_pose_x = odom_pose.pose.pose.position.x;
     robot_pose_y = odom_pose.pose.pose.position.y;
     robot_pose_z = odom_pose.pose.pose.position.z;

     robot_pose_yaw = potbot_lib::utility::get_Yaw(odom_pose.pose.pose.orientation);
    

}

std::vector<int> within_range_detection(){
    std::vector<int> in_range_ids;
    for(const auto& marker_pose : marker_positions)
    {
        double dx = robot_pose_x - marker_pose.position.x;
        double dy = robot_pose_y - marker_pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if(distance > radius){
            return false;
        }

        double angle = std::atan2(dy, dx);
        if (angle < 0){
            angle += 2 * M_PI;
        }

        double start_angle = robot_pose_yaw - angle_area / 2;
        double end_angle = robot_pose_yaw + angle_area / 2;

        if (start_angle < 0) start_angle += 2 * M_PI;
        if (end_angle >= 2 * M_PI) end_angle -= 2 * M_PI;
        
        if (start_angle < end_angle) {
            if( start_angle <= angle && angle <= end_angle){
                return true;
            };
        } else {
            if (start_angle <= angle || angle <= end_angle){
                return true;
            };
        }
    } 
    return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_Visualization");
    ros::NodeHandle nh;
    
    ros::Rate rate(50.0);

    // サブスクライバの作成
    ros::Subscriber sub_marker = nh.subscribe("marker", 1000, Marker_callback);
    ros::Subscriber sub_robot_pose = nh.subscribe("odom", 1000, RobotPose_Callback);

    while(ros::ok())
    {
        // ROS_INFO("Position: x=%.2f, y=%.2f, z=%.2f", robot_pose_x, robot_pose_y, robot_pose_z);
        // ROS_INFO("Orientation (RPY): yaw=%.2f", robot_pose_yaw);
        
        if (within_range_detection()) {
        std::cout << "All markers are within range and angle." << std::endl;
        // 範囲内にある場合の処理
        } else {
        std::cout << "At least one marker is out of range or angle." << std::endl;
        // 範囲外にある場合の処理
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}