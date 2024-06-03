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
#include <nav_msgs/Odometry.h>
#include <algorithm> 
#include <fstream>

//構造体の定義(本来はincludeを作成してやるべき)--------------------------------------------------------------------------------------------------------------------------------------------------
struct ParticlePosition {
    double x;
    double y;
    double yaw;
};

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//各記録用(ofstream定義)------------------------------------------------------------------------------------------------------------------------------------------------------------------------
std::ofstream record_start_angle("/home/ros/catkin_ws/src/user/src/date/simulator/record_start_angle_txt"); //観測用カメラの測定開始角度
std::ofstream record_end_angle("/home/ros/catkin_ws/src/user/src/date/simulator/record_end_angle_txt"); //観測用カメラの測定最終角度
std::ofstream record_robot_pose_yaw("/home/ros/catkin_ws/src/user/src/date/simulator/record_robot_pose_yaw_txt"); //ロボットの姿勢角度(観測では上記の2つの真ん中になっていれば良い)
std::ofstream record_angle("/home/ros/catkin_ws/src/user/src/date/simulator/record_angle_txt"); //ロボットとマーカーの角度(atan関数が機能しているかの確認)
std::ofstream particle_position("/home/ros/catkin_ws/src/user/src/date/simulator/particle_position_txt"); //各パーティクルの位置(動作モデル後)
std::ofstream marker_particle_information("/home/ros/catkin_ws/src/user/src/date/simulator/ marker_particle_information_txt"); //パーティクルとマーカー間の距離と角度(観測モデル使用情報)
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//各グローバル変数(定義)-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
std::vector<ParticlePosition> particle_positions; //パーティクル位置定義(動作モデル後)
std::vector<geometry_msgs::Pose> marker_positions; //マーカー位置定義
std::vector<int> marker_ids;; //マーカーid
std::vector<double> distance; //各マーカーと各パーティクルの距離
std::vector<double> angle; //各マーカーと各パーティクルの角度誤差
//std::vector<double> particle_position_x; //各パーティクルのx座標
//std::vector<double> particle_position_y; //各パーティクルのx座標
//std::vector<double> particle_position_yaw; //各パーティクルのx座標

double robot_pose_x = 0.0, robot_pose_y = 0.0, robot_pose_z = 0.0, robot_pose_yaw = 0.0; //ロボット位置の真値(定義)
double particle_position_x = 0.0, particle_position_y = 0.0, particle_position_yaw = 0.0; //パーティクルの位置(使ってない)
double dis_X = 0.0, dis_Y = 0.0; //各パーティクルの各マーカー間のX軸、Y軸誤差
double radius = 3.0, start_angle = 0.0, end_angle = 0.0, angle_area = 85.2 * M_PI / 180; //ロボットの観測範囲の定義
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//マーカーのコールバック関数---------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Marker_callback(const visualization_msgs::MarkerArray& marker_array)
{   
    marker_positions.clear(); //マーカの
    marker_ids.clear();

	for(const auto& marker : marker_array.markers)
    {
        geometry_msgs::Pose marker_pose = marker.pose;
        
        marker_ids.push_back(marker.id);
        marker_positions.push_back(marker_pose);

        // ROS_INFO("Marker ID: %d, Position: [x: %f, y: %f, z: %f]", 
        //           marker.id, marker_pose.position.x, marker_pose.position.y, marker_pose.position.z);
    }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//ロボットのコールバック関数----------------------------------------------------------------------------------------------------------------------------------------------------------------------
void RobotPose_Callback(const nav_msgs::Odometry& odom_pose)
{
    
     robot_pose_x = odom_pose.pose.pose.position.x;
     robot_pose_y = odom_pose.pose.pose.position.y;
     robot_pose_z = odom_pose.pose.pose.position.z;

     robot_pose_yaw = potbot_lib::utility::get_Yaw(odom_pose.pose.pose.orientation);
    
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//パーティクルのコールバック関数-------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Particle_Callback(const geometry_msgs::PoseArray& particle_pose)
{
    particle_positions.clear();
     //particle_position.clear()
     for(const auto& particle_pose : particle_pose.poses)
    {
        ParticlePosition particle_position;
        particle_position.x = particle_pose.position.x;
        particle_position.y = particle_pose.position.y;
        particle_position.yaw = potbot_lib::utility::get_Yaw(particle_pose.orientation);
        // ROS_INFO(" Position: [x: %f, y: %f, z: %f]", 
        //            particle_pose.position.x, particle_pose.position.y, particle_pose.position.z);
        
        particle_positions.push_back(particle_position);

        //particle_position << "particle_position.x" << particle_pose.position.x << "particle_position.y" << particle_pose.position.y << "particle_position.yaw" << particle_position_yaw << std::endl;
    }
  
 }
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//範囲内マーカー観測プロセス-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
std::vector<int> within_range_detection(){
    std::vector<int> in_range_ids;
    for(size_t i = 0; i < marker_positions.size(); ++i)
    {
        const auto& marker_pose = marker_positions[i];
        double dx = marker_pose.position.x - robot_pose_x;
        double dy = marker_pose.position.y - robot_pose_y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if(distance > radius){
            continue;
        }

        double angle = std::atan2(dy, dx);
        if (angle < 0){
            angle += 2 * M_PI;
        }
        
        record_angle << angle  << std :: endl;

        double start_angle = robot_pose_yaw - angle_area / 2;
        double end_angle = robot_pose_yaw + angle_area / 2;
        
        record_start_angle << start_angle  << std :: endl;
        record_end_angle << end_angle  << std :: endl;
        record_robot_pose_yaw << robot_pose_yaw   << std :: endl;

        if (start_angle < 0) start_angle += 2 * M_PI;
        if (end_angle >= 2 * M_PI) end_angle -= 2 * M_PI;
        
        if((start_angle < end_angle && start_angle <= angle && angle <= end_angle) || (start_angle >= end_angle && (start_angle <= angle || angle <= end_angle)))
        {
            in_range_ids.push_back(marker_ids[i]);
        }
    } 
    return in_range_ids;
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//観測モデルの実装----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Visualization_mode(const std::vector<int>& in_range_ids)
{
    distance.clear();
    angle.clear();

    for (size_t i = 0; i < marker_positions.size(); ++i)
    {
        if (std::find(in_range_ids.begin(), in_range_ids.end(), marker_ids[i]) != in_range_ids.end())
        {
            const auto& marker = marker_positions[i];
            for (size_t j = 0; j < particle_positions.size(); ++j)
            {
                const auto & particle = particle_positions[j]; 

                dis_X = marker.position.x - particle.x;
                dis_Y = marker.position.y - particle.y;

                double dis = sqrt(dis_X * dis_X + dis_Y*dis_Y);
                double ang = atan2(dis_Y , dis_X);
                if(ang < 0){
                    ang += 2 * M_PI;
                }
                
                distance.push_back(dis);
                angle.push_back(ang);

                std::cout << "Marker ID " << marker_ids[i] << " and Particle " << j
                          << " distance: (d: " << dis << ", angle: (a: " << ang << std::endl;
                
                marker_particle_information << "Marker ID " << marker_ids[i] << " and Particle " << j
                                            << " distance: (d: " << dis << ", angle: (a: " << ang << std::endl;
            }
        }
    }
}   
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_Visualization");
    ros::NodeHandle nh;
    
    ros::Rate rate(50.0);

    // サブスクライバの作成
    ros::Subscriber sub_marker = nh.subscribe("marker", 1000, Marker_callback);
    ros::Subscriber sub_robot_pose = nh.subscribe("odom", 1000, RobotPose_Callback);
    ros::Subscriber sub_particle_pose = nh.subscribe("particles", 1000, Particle_Callback);

    while(ros::ok())
    {
        //ROS_INFO("Position: x=%.2f, y=%.2f, z=%.2f", robot_pose_x, robot_pose_y, robot_pose_z);
        //ROS_INFO("Orientation (RPY): yaw=%.2f", robot_pose_yaw);
        std::vector<int> in_range_ids = within_range_detection();

        if (! in_range_ids.empty()){
            std::cout << "Markers within range and angle ";
            for(int id : in_range_ids){
                std::cout << id << "  ";
            }
            std::cout << std::endl;

            for(size_t i = 0; i < marker_positions.size(); ++i) {
                if(std::find(in_range_ids.begin(), in_range_ids.end(), marker_ids[i]) != in_range_ids.end()){
                    Visualization_mode(in_range_ids);
                }
            }
        }else {
            std::cout << "No markers are within range or angle." << std::endl;
        }

        Visualization_mode(in_range_ids); 
        // if (within_range_detection()) {
        // std::cout << "All markers are within range and angle." << std::endl;
        // // 範囲内にある場合の処理
        // } else {
        // std::cout << "At least one marker is out of range or angle." << std::endl;
        // // 範囲外にある場合の処理
        // }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}