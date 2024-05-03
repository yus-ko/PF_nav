#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <potbot_lib/Utility.h>

#include <string>
#include <math.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "info_marker_publisher1");
  ros::NodeHandle nh;

  // publisher
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 1);

  ros::NodeHandle n("~");
  std::vector<geometry_msgs::Pose> marker_pose;
  std::vector<geometry_msgs::Vector3> marker_scale;
  size_t id = 0;
  while (1)
  {
    std::string param_name = "marker" + std::to_string(id);
    if(!n.hasParam(param_name + "/x")) break;
    double x,y,yaw_deg;
    n.getParam(param_name + "/x", x);
    n.getParam(param_name + "/y", y);
    n.getParam(param_name + "/yaw", yaw_deg);
    marker_pose.push_back(potbot_lib::utility::get_Pose(x,y,0,0,0,yaw_deg/180.0*M_PI));
    id++;
  }

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    double t = marker.header.stamp.toSec();

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.2;
    
    marker.pose = marker_pose[0];
    marker.color = potbot_lib::color::get_msg(potbot_lib::color::LIGHT_BLUE);
    marker_pub.publish(marker);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
