#ifndef TRAJECTORY_PUBLISHER_SAVER_HPP
#define TRAJECTORY_PUBLISHER_SAVER_HPP

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <vector>
#include "trajectory_visualizer/SaveTrajectory.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

struct TrajectoryPoint {
    double x, y, yaw, time;
};

class TrajectoryPublisherSaver {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher trajectory_pub_;
    ros::ServiceServer save_service_;

    std::vector<TrajectoryPoint> trajectory_;
    std::string namespace_param_;
    std::string odom_frame_;
    std::string odom_topic_;
    std::string marker_topic_;
    float marker_interval_;
    ros::Time current_time, prev_time;
    double current_x, prev_x;
    double current_y, prev_y;
    double current_yaw, prev_yaw;
    double position_tolerance_;
    double yaw_tolerance_;

public:
    TrajectoryPublisherSaver();
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void publishMarkers();
    bool saveTrajectory(trajectory_visualizer::SaveTrajectory::Request &req,
                        trajectory_visualizer::SaveTrajectory::Response &res);
    void saveAsCSV(const std::string& filename, const std::vector<TrajectoryPoint>& data);
    geometry_msgs::Quaternion yawToQuaternion(double yaw);
    double quaternionToYaw(const geometry_msgs::Quaternion quat_msg);
};

#endif  // TRAJECTORY_PUBLISHER_SAVER_HPP
