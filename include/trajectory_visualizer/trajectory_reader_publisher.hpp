#ifndef TRAJECTORY_READER_PUBLISHER_HPP
#define TRAJECTORY_READER_PUBLISHER_HPP

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>

struct TrajectoryPoint {
    double time, x, y, yaw;
};

class TrajectoryReaderPublisher {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher trajectory_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string trajectory_file_;
    std::string odom_frame_;
    std::string marker_topic_;

public:
    TrajectoryReaderPublisher();
    void publishTrajectory();
    std::vector<TrajectoryPoint> loadTrajectory(const std::string& filename);
    geometry_msgs::Quaternion yawToQuaternion(double yaw);
};

#endif // TRAJECTORY_READER_PUBLISHER_HPP
