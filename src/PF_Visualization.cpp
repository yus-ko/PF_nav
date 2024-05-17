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

double robot_pose_x = 0.0, robot_pose_y = 0.0, robot_pose_z = 0.0, robot_pose_yaw = 0.0;
double radius = 0.0, start_angle = 0.0, end_angle = 0.0;

// struct Point {
//     double x, y;
// };

// struct Sector {
//     Point center;     // 中心点
//     double radius;    // 半径
//     double start_angle; // 開始角度 (ラジアン)
//     double end_angle;   // 終了角度 (ラジアン)

//     Sector(Point c, double r, double start, double end)
//         : center(c), radius(r), start_angle(start), end_angle(end) {}
// };

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
    
    double robot_pose_x = odom_pose.pose.pose.position.x;
    double robot_pose_y = odom_pose.pose.pose.position.y;
    double robot_pose_z = odom_pose.pose.pose.position.z;

    double robot_pose_yaw = potbot_lib::utility::get_Yaw(odom_pose.pose.pose.orientation);
    

}

bool within_range_detection(){


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_Visualization");
    ros::NodeHandle nh;

    // サブスクライバの作成
    ros::Subscriber sub_marker = nh.subscribe("marker", 1000, Marker_callback);
    ros::Subscriber sub_robot_pose = nh.subscribe("odom", 1000, RobotPose_Callback);

    ros::spin();
    
    ROS_INFO("Position: x=%.2f, y=%.2f, z=%.2f", robot_pose_x, robot_pose_y, robot_pose_z);
    ROS_INFO("Orientation (RPY): yaw=%.2f", robot_pose_yaw);


    return 0;
}