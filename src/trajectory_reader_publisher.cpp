#include "trajectory_visualizer/trajectory_reader_publisher.hpp"

TrajectoryReaderPublisher::TrajectoryReaderPublisher() 
    : private_nh_("~"), tf_listener_(tf_buffer_) {
    
    // Load parameters
    private_nh_.param<std::string>("trajectory_file", trajectory_file_, "default.csv");
    private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
    private_nh_.param<std::string>("marker_topic", marker_topic_, "trajectory_markers");

    trajectory_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 10);
    
    ROS_INFO("Trajectory Reader Publisher Initialized.");
    publishTrajectory();
}

void TrajectoryReaderPublisher::publishTrajectory() {
    std::vector<TrajectoryPoint> trajectory = loadTrajectory(trajectory_file_);
    if (trajectory.empty()) {
        ROS_ERROR("No trajectory data loaded.");
        return;
    }

    visualization_msgs::MarkerArray marker_array;
    int id = 0;

    for (const auto& point : trajectory) {
        geometry_msgs::PointStamped original_point, transformed_point;
        original_point.header.frame_id = "map";  // Assuming saved data is in "map" frame
        original_point.point.x = point.x;
        original_point.point.y = point.y;
        original_point.point.z = 0.0;

        try {
            geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
                odom_frame_, "map", ros::Time(0), ros::Duration(1.0)
            );
            tf2::doTransform(original_point, transformed_point, transform_stamped);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Could not transform point: %s", ex.what());
            continue;
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = odom_frame_;
        marker.header.stamp = ros::Time::now();
        marker.ns = "trajectory";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = point.x;
        marker.pose.position.y = point.y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation = yawToQuaternion(point.yaw);
        marker.scale.x = 0.08;
        marker.scale.y = 0.02;
        marker.scale.z = 0.04;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0);
        
        marker_array.markers.push_back(marker);
    }

    trajectory_pub_.publish(marker_array);
    ROS_INFO("Published transformed trajectory.");
}

std::vector<TrajectoryPoint> TrajectoryReaderPublisher::loadTrajectory(const std::string& filename) {
    std::vector<TrajectoryPoint> trajectory;
    std::ifstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open trajectory file: %s", filename.c_str());
        return trajectory;
    }

    std::string line;
    std::getline(file, line); // Skip header
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        TrajectoryPoint point;
        char comma;
        if (ss >> point.time >> comma >> point.x >> comma >> point.y >> comma >> point.yaw) {
            trajectory.push_back(point);
        }
    }
    file.close();
    return trajectory;
}

geometry_msgs::Quaternion TrajectoryReaderPublisher::yawToQuaternion(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    geometry_msgs::Quaternion quat_msg;
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();
    quat_msg.w = q.w();

    return quat_msg;
}
