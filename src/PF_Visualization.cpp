#include <PF_nav/PF_Visualization.h>

PFVisualization::PFVisualization(/* args */)
{
    ros::NodeHandle n("~");
	// n.getParam("particle_num", control_frequency); //launchファイルから取得周期(50.0)

    // record_end_angle_ = std::ofstream("/home/ros/catkin_ws/src/user/src/date/simulator/record_end_angle__txt");
    // record_robot_pose_yaw = std::ofstream("/home/ros/catkin_ws/src/user/src/date/simulator/record_robot_pose_yaw_txt");
    // record_angle = std::ofstream("/home/ros/catkin_ws/src/user/src/date/simulator/record_angle_txt");
    // particle_position = std::ofstream("/home/ros/catkin_ws/src/user/src/date/simulator/particle_position_txt");
    // marker_particle_information = std::ofstream("/home/ros/catkin_ws/src/user/src/date/simulator/ marker_particle_information_txt");
    // particle_weight = std::ofstream("/home/ros/catkin_ws/src/user/src/date/simulator/ particle_weight_txt");
    // particle_Norm_weight = std::ofstream("/home/ros/catkin_ws/src/user/src/date/simulator/ particle_Norm_weight_txt");

    ros::NodeHandle nh;
    int particle_num = 100;
	double norm_noise_mean_linear_velocity = 0;
	double norm_noise_variance_linear_velocity = 0.1;
	double norm_noise_mean_angular_velocity = 0;
	double norm_noise_variance_angular_velocity = 0.1;
	
	n.getParam("particle_num", particle_num);
	n.getParam("norm_noise_mean_linear_velocity", norm_noise_mean_linear_velocity);
	n.getParam("norm_noise_variance_linear_velocity", norm_noise_variance_linear_velocity);
	n.getParam("norm_noise_mean_angular_velocity", norm_noise_mean_angular_velocity);
	n.getParam("norm_noise_variance_angular_velocity", norm_noise_variance_angular_velocity);

	pub_particles_ = nh.advertise<geometry_msgs::PoseArray>("particles", 1);
	pub_particles_state_ = nh.advertise<potbot_msgs::ObstacleArray>("particles_state", 1);

	// ros::Subscriber sub_inipose				= nh.subscribe("initialpose",1,inipose_callback);

    pub_estimated_robot_ = nh.advertise<nav_msgs::Odometry>("odom/estimated", 1);

    // サブスクライバの作成
    sub_marker_ = nh.subscribe("marker", 1000, &PFVisualization::markerCallback,this);
    sub_robot_pose_ = nh.subscribe("odom", 1000, &PFVisualization::robotPoseCallback,this);
    // sub_particle_pose_ = nh.subscribe("particles", 1000, &PFVisualization::particleCallback,this);

    
    // geometry_msgs::PoseArray particles_msg;
	// // particles_msg.header.frame_id = robot_pose.header.frame_id;
	// potbot_msgs::ObstacleArray particle_state_msg;
	// particle_state_msg.header.frame_id = robot_pose.header.frame_id;
	// for (size_t i = 0; i < particle_num; i++) 
	// {
	// 	particles_msg.poses.push_back(robot_pose.pose.pose);
	// 	potbot_msgs::Obstacle p;
	// 	p.pose = robot_pose.pose.pose;
	// 	particle_state_msg.data.push_back(p);
	// };

	// for (auto& robo : g_robot)
	// {
	// 	// robo.deltatime = 1.0/control_frequency;
	// 	robo.set_msg(robot_pose);
	// }

    particles_.resize(particle_num);

	std::random_device rd;
    std::default_random_engine generator(rd());
    std::normal_distribution<double> distribution_linear_velocity(norm_noise_mean_linear_velocity, sqrt(norm_noise_variance_linear_velocity));
	std::normal_distribution<double> distribution_angular_velocity(norm_noise_mean_angular_velocity, sqrt(norm_noise_variance_angular_velocity));
}

void PFVisualization::markerCallback(const visualization_msgs::MarkerArray& marker_array)
{   
    sebscribed_landmark_pose_ = true;
    
    marker_positions_.clear(); //マーカの
    marker_ids_.clear();

	for(const auto& marker : marker_array.markers)
    {
        geometry_msgs::Pose marker_pose = marker.pose;
        
        marker_ids_.push_back(marker.id);
        marker_positions_.push_back(marker_pose);

        // ROS_INFO("Marker ID: %d, Position: [x: %f, y: %f, z: %f]", 
        //           marker.id, marker_pose.position.x, marker_pose.position.y, marker_pose.position.z);
    }
}

//ロボットのコールバック関数
void PFVisualization::robotPoseCallback(const nav_msgs::Odometry& odom_pose)
{
    sebscribed_robot_pose_ = true;
    odom_msg_ = odom_pose;
    robot_pose_x_ = odom_pose.pose.pose.position.x;
    robot_pose_y_ = odom_pose.pose.pose.position.y;
    robot_pose_z_ = odom_pose.pose.pose.position.z;

    robot_pose_yaw_ = potbot_lib::utility::get_Yaw(odom_pose.pose.pose.orientation);
    
    if (sebscribed_landmark_pose_ && sebscribed_robot_pose_ && sebscribed_particle_pose_)
    {
        localization();
    }    
    
}

//パーティクルのコールバック関数
void PFVisualization::particleCallback(const geometry_msgs::PoseArray& particle_pose)
{
    sebscribed_particle_pose_ = true;

    particle_positions_.clear();
    //particle_position.clear()
    for(const auto& particle_pose : particle_pose.poses)
    {
        ParticlePosition particle_position;
        particle_position.x = particle_pose.position.x;
        particle_position.y = particle_pose.position.y;
        particle_position.yaw = potbot_lib::utility::get_Yaw(particle_pose.orientation);
        // ROS_INFO(" Position: [x: %f, y: %f, z: %f]", 
        //            particle_pose.position.x, particle_pose.position.y, particle_pose.position.z);
        
        particle_positions_.push_back(particle_position);

        //particle_position << "particle_position.x" << particle_pose.position.x << "particle_position.y" << particle_pose.position.y << "particle_position.yaw" << particle_position_yaw << std::endl;
    }
  
}

void PFVisualization::localization()
{

    std::vector<int> in_range_ids;
    getObservedLandmark(in_range_ids);
    initLiklihood();

    for (const auto& marker_id:in_range_ids)
    {
        getLikelihood(marker_id);
    }
    normLiklihood();

    getEstimatedRobotPose();


}

void PFVisualization::updateParticles()
{
    // for (size_t i = 1; i < particle_num+1; i++)
    // {
    //     double v_noise = v_truth + distribution_linear_velocity(generator);
    //     double omega_noise = omega_truth + distribution_angular_velocity(generator);
    //     nav_msgs::Odometry particle;
    //     g_robot[i].to_msg(particle);
    //     particle.twist.twist.linear.x = v_noise;
    //     particle.twist.twist.angular.z = omega_noise;
    //     g_robot[i].set_msg(particle);
    //     g_robot[i].update();
    //     particles_msg.poses[i-1] = particle.pose.pose;

    //     particle_state_msg.data[i-1].pose = particle.pose.pose;
    //     particle_state_msg.data[i-1].twist = particle.twist.twist;

    // }
    
    // pub_particles_.publish();
}

void PFVisualization::initLiklihood()
{
    Likelihood_.clear();
    Likelihood_.resize(particle_positions_.size());
    std::fill(Likelihood_.begin(), Likelihood_.end(), 1.0);
}

void PFVisualization::normLiklihood()
{
    double sum = std::accumulate(Likelihood_.begin(), Likelihood_.end(), 0.0);
    for(auto& val:Likelihood_) val/=sum;
}

//範囲内マーカー観測プロセス
void PFVisualization::getObservedLandmark(std::vector<int>& in_range)
{
    in_range.clear();
    for(size_t i = 0; i < marker_positions_.size(); ++i)
    {
        const auto& marker_pose = marker_positions_[i];
        double dx = marker_pose.position.x - robot_pose_x_;
        double dy = marker_pose.position.y - robot_pose_y_;
        Robot_distance_ = std::sqrt(dx * dx + dy * dy);
        
        if(Robot_distance_ > radius_){
            continue;
        }

        Robot_angle_ = robot_pose_yaw_ - std::atan2(dy, dx);
        // if (Robot_angle_ < 0){
        //     Robot_angle_ += 2 * M_PI;
        // }
        
        // record_angle << Robot_angle_  << std :: endl;

        // double start_angle_ = robot_pose_yaw_ - angle_area_ / 2;
        // double end_angle_ = robot_pose_yaw_ + angle_area_ / 2;
        
        // record_start_angle_ << start_angle_  << std :: endl;
        // record_end_angle_ << end_angle_  << std :: endl;
        // record_robot_pose_yaw << robot_pose_yaw   << std :: endl;

        // if (start_angle_ < 0) start_angle_ += 2 * M_PI;
        // if (end_angle_ >= 2 * M_PI) end_angle_ -= 2 * M_PI;
        
        // if((start_angle_ < end_angle_ && start_angle_ <= Robot_angle_ && Robot_angle_ <= end_angle_) || 
        //     (start_angle_ >= end_angle_ && (start_angle_ <= Robot_angle_ || 
        //     Robot_angle_ <= end_angle_)))
        if (abs(Robot_angle_) <= angle_area_ && Robot_distance_ <= radius_)
        {
            in_range.push_back(marker_ids_[i]);
        }
    } 
    ROS_INFO("observed landmarks num: %d", in_range.size());
}

//マーカー、パーティクル間誤差、尤度計算(距離、角度)
void PFVisualization::getLikelihood(size_t marker_id)
{
    const auto& marker = marker_positions_[marker_id];

    for (size_t j = 0; j < particle_positions_.size(); ++j)
    {
        const auto & particle = particle_positions_[j]; 

        dis_X_ = marker.position.x - particle.x;
        dis_Y_ = marker.position.y - particle.y;
        
        double particle_distance = sqrt(dis_X_ * dis_X_ + dis_Y_ * dis_Y_);
        double particle_angle = particle.yaw - atan2(dis_Y_ , dis_X_);
        // if(particle_angle < 0){
        //     particle_angle += 2 * M_PI;
        // }
        
        double w_dis = 1/(sqrt(2 * M_PI * dis_var_))*exp(-((abs(Robot_distance_)-abs(particle_distance))*(abs(Robot_distance_)-abs(particle_distance)))/(2*dis_var_))+1e-100; 

        double w_ang =1/(sqrt(2 * M_PI * ang_var_))*exp(-(( Robot_angle_ - (- particle_angle - particle.yaw)) * ( Robot_angle_ - (- particle_angle - particle.yaw))) / (2 * ang_var_))+1e-100;
        
        if(Robot_angle_ * particle_angle > 0 && particle_angle > 1.57)
        {
            w_ang = 1/(sqrt(2 * M_PI * ang_var_))*exp(-(( Robot_angle_ - (- particle_angle - (particle.yaw - 2 * M_PI))) * ( Robot_angle_ - (- particle_angle - (particle.yaw - 2 * M_PI)))) / (2 * M_PI * ang_var_))+1e-100;
        }else if (Robot_angle_ * particle_angle > 0 && particle_angle < -1.57)
        {
            w_ang = 1/(sqrt(2 * M_PI * ang_var_))*exp(-(( Robot_angle_ - (- particle_angle - (particle.yaw + 2 * M_PI))) * ( Robot_angle_ - (- particle_angle - (particle.yaw + 2 * M_PI)))) / (2 * M_PI * ang_var_))+1e-100;
        }
        

        double w_dis_log = log10(w_dis);
        double w_ang_log = log10(w_ang);

        double weight = exp(w_dis_log + w_ang_log);
        
        Likelihood_[j]*=weight;

        //std::cout << "distance_weight" <<  w_dis << "angle_weight" << w_ang << "total_wight" << total_weight_ << :: std::endl;

        // std::cout << "Marker ID " << marker_ids_[i] << " and Particle " << j
        //           << " distance: (d: " << dis << ", angle: (a: " << ang << std::endl;
        // particle_weight << "distance_weight" << " " <<  w_dis_log << " " << "angle_weight" << " " << w_ang_log << " " << "wight" << " " << weight << " " << "total_wight" << total_weight_ << :: std::endl;

        // marker_particle_information << " " << "Marker ID " << " " << marker_ids_[marker_id] << " " << " and Particle " << " " << j
        //                             << " distance: (d: " << " " << particle_distance << " " << ", angle: (a: " << " " << particle_angle << std::endl;

        // for (const auto& info : Error_Information_) 
        // {
        // std::cout << "Angle: " << info.ang << ", Distance: " << info.dis << std::endl;
        // }
    }

    ROS_INFO_STREAM("Likelihood_ size: " << Likelihood_.size());
    ROS_INFO_STREAM("particle size: " <<  particle_positions_.size());
}   
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//重みの最適化過程、自己位置推定過程、リサンプリング過程--------------------------------------------------------------------------------------------------------------------------------------------
void PFVisualization::getEstimatedRobotPose()
{
    double Norm_total_weight = std::accumulate(Likelihood_.begin(), Likelihood_.end(), 0.0);
    double Particle_Est_RobotX = 0.0;
    double Particle_Est_RobotY = 0.0;
    double Particle_Est_RobotYaw = 0.0;

    ROS_INFO("Starting Localization_mode");
    
    for (size_t j = 0; j < particle_positions_.size(); ++j)
    {
        const auto & particle = particle_positions_[j];
        const auto & w = Likelihood_[j];
            
        // std::cout << " 正規化前尤度 " << w << std :: endl;


        Particle_Est_RobotX += particle.x * w;
        Particle_Est_RobotY += particle.y * w;
        Particle_Est_RobotYaw += particle.yaw * w;

        // std::cout << " 正規尤度 " << Norm_weight << std :: endl; 
        // particle_Norm_weight << "weight" << " " << w << " " << "total_weight" << " " << total_weight << " " << "Norm_weight" << " " << Norm_weight << " " << "Norm_total_weight" << " " << Norm_total_weight << std :: endl;

    }
    
    // std::cout << "=========" << std::endl;
    // std::cout << "Particle_Est_RobotX=" <<Particle_Est_RobotX<< std::endl;
    // std::cout << "Particle_Est_RobotY=" <<Particle_Est_RobotY<< std::endl;
    // std::cout << "Particle_Est_RobotTH=" <<Particle_Est_RobotYaw<< std::endl;
    ROS_INFO_STREAM("Norm total weight: " << Norm_total_weight <<
                    " EstX: " << Particle_Est_RobotX <<
                    " EstY: " << Particle_Est_RobotY <<
                    " EstTh: " << Particle_Est_RobotYaw);

    nav_msgs::Odometry est_msg;
    est_msg.header.stamp = ros::Time::now();
    est_msg.header.frame_id = odom_msg_.header.frame_id;
    est_msg.child_frame_id = odom_msg_.child_frame_id;
    est_msg.pose.pose = potbot_lib::utility::get_Pose(Particle_Est_RobotX,Particle_Est_RobotY,0,0,0,Particle_Est_RobotYaw);
    pub_estimated_robot_.publish(est_msg);
}

//リサンプリング(鈴木ver)
void PFVisualization::getResamplingRobotPose1(std::vector<double>& step_sum_weight_)
{
    step_sum_weight_.clear();

    for ( size_t i = 0; i < particle_positions_.size(); ++i)
    {
       double step_weight = 0.0;
       step_weight += Likelihood_[i];
       step_sum_weight_.push_back(step_weight);
    }

    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<double> distr(0,step_sum_weight_[ particle_positions_.size() - 1]);
    double darts = distr(eng);

    int resam_num = 0;

}

//リサンプリング(赤井先生ver)
// void PFVisualization::getResamplingRobotPose2(std::vector<double>& step_sum_weight_)
// {
//     step_sum_weight_.clear();
// }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "marker_Visualization");
    PFVisualization pfv;

    // while (ros::ok())
    // {
    //     pfv.resampling();
    //     ros::spinOnce();
    // }
    
    // ros::spin();

    return 0;
}