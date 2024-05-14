#include <ros/ros.h>
#include <potbot_lib/DiffDriveController.h>
#include <potbot_msgs/ObstacleArray.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Pose.h>
#include <PF_nav/PF_navConfig.h>
#include <geometry_msgs/PoseArray.h>
#include <random>
#include <Visualization_msgs>

std::vector<geometry_msgs::Pose> marker_positions;


void Marker_callback(const visualization_msgs::MarkerArray& marker)
{
	for(const auto& marker : marker.markers)
    {
        geometry_msgs::Pose marker_pose = marker.pose;

        marker_positions.push_back(marker_pose);

        Ros_INFO("Marker ID: %d, Position: [x: %f, y: %f, z: %f]", 
                  marker.id, marker_pose.position.x, marker_pose.position.y, marker_pose.position.z);
    }
}

int main(int argc, char argv)
{
    ros::init(argc, argv, "marker_position_extractor");
    ros::NodeHandle n;

    // サブスクライバの作成
    ros::Subscriber sub = n.subscribe("marker_topic", 1000, Marker_callback);

    // ROSのイベントループを開始
    ros::spin();

    return 0;
}