#include <ros/ros.h>
#include <potbot_lib/DiffDriveController.h>
#include <potbot_msgs/ObstacleArray.h>
#include <dynamic_reconfigure/server.h>
#include <PF_nav/PF_navConfig.h>
#include <geometry_msgs/PoseArray.h>
#include <random>


std::vector<potbot_lib::Controller::DiffDriveController> g_robot;

void inipose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	nav_msgs::Odometry ini_pose;
	ini_pose.header.frame_id = "map" ;
	ini_pose.pose = msg.pose;
	for (auto& robo : g_robot) robo.set_msg(ini_pose);
}

void goal_callback(const geometry_msgs::PoseStamped& msg)
{
	g_robot[0].set_target(	msg.pose.position.x,
							msg.pose.position.y,
							potbot_lib::utility::get_Yaw(msg.pose.orientation));
}

void param_callback(const PF_nav::PF_navConfig& param, uint32_t level)
{
	g_robot[0].set_gain(	param.gain_p,
							param.gain_i,
							param.gain_d);

	g_robot[0].set_margin(	param.stop_margin_angle,
							param.stop_margin_distance);

	g_robot[0].set_limit(	param.max_linear_velocity,
							param.max_angular_velocity);
}

int main(int argc,char **argv){
	ros::init(argc,argv,"controller_tutorial");

	ros::NodeHandle n("~");

	double control_frequency = 50.0;
	int particle_num = 100;
	double norm_noise_mean_linear_velocity = 0;
	double norm_noise_variance_linear_velocity = 0.1;
	double norm_noise_mean_angular_velocity = 0;
	double norm_noise_variance_angular_velocity = 0.1;
	
	n.getParam("control_frequency", control_frequency);
	n.getParam("particle_num", particle_num);
	n.getParam("norm_noise_mean_linear_velocity", norm_noise_mean_linear_velocity);
	n.getParam("norm_noise_variance_linear_velocity", norm_noise_variance_linear_velocity);
	n.getParam("norm_noise_mean_angular_velocity", norm_noise_mean_angular_velocity);
	n.getParam("norm_noise_variance_angular_velocity", norm_noise_variance_angular_velocity);

	g_robot.resize(particle_num+1);

	ros::NodeHandle nh;

	ros::Publisher pub_odom					= nh.advertise<nav_msgs::Odometry>("odom", 1);
	ros::Publisher pub_particles			= nh.advertise<geometry_msgs::PoseArray>("particles", 1);
	ros::Publisher pub_particles_state		= nh.advertise<potbot_msgs::ObstacleArray>("particles_state", 1);

	ros::Subscriber sub_inipose				= nh.subscribe("initialpose",1,inipose_callback);
	ros::Subscriber sub_goal				= nh.subscribe("move_base_simple/goal",1,goal_callback);

	dynamic_reconfigure::Server<PF_nav::PF_navConfig> server;
	dynamic_reconfigure::Server<PF_nav::PF_navConfig>::CallbackType f;
	f = boost::bind(param_callback, _1, _2);
	server.setCallback(f);
	
	ros::Rate rate(control_frequency);

	nav_msgs::Odometry robot_pose;
	robot_pose.header.frame_id = "map";
	robot_pose.pose.pose = potbot_lib::utility::get_Pose(0,0,0,0,0,0);	//ロボット初期位置 x y z r p y

	geometry_msgs::PoseArray particles_msg;
	particles_msg.header.frame_id = robot_pose.header.frame_id;
	potbot_msgs::ObstacleArray particle_state_msg;
	particle_state_msg.header.frame_id = robot_pose.header.frame_id;
	for (size_t i = 0; i < particle_num; i++) 
	{
		particles_msg.poses.push_back(robot_pose.pose.pose);
		potbot_msgs::Obstacle p;
		p.pose = robot_pose.pose.pose;
		particle_state_msg.data.push_back(p);
	};

	for (auto& robo : g_robot)
	{
		robo.deltatime = 1.0/control_frequency;;
		robo.set_msg(robot_pose);
	}

	std::random_device rd;
    std::default_random_engine generator(rd());
    std::normal_distribution<double> distribution_linear_velocity(norm_noise_mean_linear_velocity, sqrt(norm_noise_variance_linear_velocity));
	std::normal_distribution<double> distribution_angular_velocity(norm_noise_mean_angular_velocity, sqrt(norm_noise_variance_angular_velocity));

	while (ros::ok())
	{

		g_robot[0].pid_control();
		g_robot[0].update();
		
		g_robot[0].to_msg(robot_pose);

		particles_msg.header = robot_pose.header;
		particle_state_msg.header = robot_pose.header;
		double v_truth = robot_pose.twist.twist.linear.x;
		double omega_truth = robot_pose.twist.twist.angular.z;
		if (v_truth != 0 || omega_truth != 0)
		{
			for (size_t i = 1; i < particle_num+1; i++)
			{
				double v_noise = v_truth + distribution_linear_velocity(generator);
				double omega_noise = omega_truth + distribution_angular_velocity(generator);
				nav_msgs::Odometry particle;
				g_robot[i].to_msg(particle);
				particle.twist.twist.linear.x = v_noise;
				particle.twist.twist.angular.z = omega_noise;
				g_robot[i].set_msg(particle);
				g_robot[i].update();
				particles_msg.poses[i-1] = particle.pose.pose;

				particle_state_msg.data[i-1].pose = particle.pose.pose;
				particle_state_msg.data[i-1].twist = particle.twist.twist;

			}
		}

		pub_odom.publish(robot_pose);
		pub_particles.publish(particles_msg);
		pub_particles_state.publish(particle_state_msg);
        
		ros::spinOnce();
		rate.sleep();
		
	}

	return 0;
}