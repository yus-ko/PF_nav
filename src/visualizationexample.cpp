#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <cmath>
#include <vector>

// 構造体の定義
struct Point {
    double x, y;
};

struct Sector {
    Point center;     // 中心点
    double radius;    // 半径
    double start_angle; // 開始角度 (ラジアン)
    double end_angle;   // 終了角度 (ラジアン)

    Sector(Point c, double r, double start, double end)
        : center(c), radius(r), start_angle(start), end_angle(end) {}
};

struct Robot {
    Point position;    // ロボットの現在位置
    double direction;  // ロボットの進行方向 (ラジアン)

    Robot(double x = 0, double y = 0, double dir = 0)
        : position({x, y}), direction(dir) {}

    void updatePose(const geometry_msgs::Pose& pose) {
        position.x = pose.position.x;
        position.y = pose.position.y;
        direction = tf::getYaw(pose.orientation);
    }

    Sector getDetectionSector(double radius, double angle_span) const {
        double start_angle = direction - angle_span / 2.0;
        double end_angle = direction + angle_span / 2.0;

        // 角度を0〜2πの範囲に正規化
        if (start_angle < 0) start_angle += 2 * M_PI;
        if (end_angle >= 2 * M_PI) end_angle -= 2 * M_PI;

        return Sector(position, radius, start_angle, end_angle);
    }
};

bool isPointInSector(const Point& point, const Sector& sector) {
    // 中心点からの距離を計算
    double dx = point.x - sector.center.x;
    double dy = point.y - sector.center.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // 半径以内にあるかを確認
    if (distance > sector.radius) {
        return false;
    }

    // 角度を計算
    double angle = std::atan2(dy, dx);
    if (angle < 0) {
        angle += 2 * M_PI;
    }

    // 角度が範囲内にあるかを確認
    if (sector.start_angle < sector.end_angle) {
        return sector.start_angle <= angle && angle <= sector.end_angle;
    } else {
        // 角度範囲が 0 を跨ぐ場合
        return sector.start_angle <= angle || angle <= sector.end_angle;
    }
}

// グローバル変数
Robot robot;
std::vector<Point> obstacles;

// コールバック関数
void robotPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    robot.updatePose(*msg);
}

void obstaclesCallback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    obstacles.clear();
    for (const auto& pose : msg->poses) {
        Point obstacle = {pose.position.x, pose.position.y};
        obstacles.push_back(obstacle);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_sector_detection");
    ros::NodeHandle nh;

    // サブスクライバの作成
    ros::Subscriber robot_pose_sub = nh.subscribe("robot_pose", 1000, robotPoseCallback);
    ros::Subscriber obstacles_sub = nh.subscribe("obstacles", 1000, obstaclesCallback);

    // センサ範囲の設定
    double sensor_radius = 5.0;
    double sensor_angle_span = M_PI / 2.0; // 90度

    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce(); // コールバックを処理

        Sector detection_sector = robot.getDetectionSector(sensor_radius, sensor_angle_span);

        for (const auto& obstacle : obstacles) {
            if (isPointInSector(obstacle, detection_sector)) {
                ROS_INFO("Obstacle at (%f, %f) is within the detection sector.", obstacle.x, obstacle.y);
            } else {
                ROS_INFO("Obstacle at (%f, %f) is outside the detection sector.", obstacle.x, obstacle.y);
            }
        }

        rate.sleep();
    }

    return 0;
}
