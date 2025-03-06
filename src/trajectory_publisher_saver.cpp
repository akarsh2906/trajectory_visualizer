#include "trajectory_visualizer/trajectory_publisher_saver.hpp"

TrajectoryPublisherSaver::TrajectoryPublisherSaver() : private_nh_("~") {
    // Get parameters
    private_nh_.param<std::string>("namespace", namespace_param_, "");
    private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
    private_nh_.param<std::string>("odom_topic", odom_topic_, "odom");
    private_nh_.param<std::string>("marker_topic", marker_topic_, "trajectory_markers");
    private_nh_.param<float>("marker_interval", marker_interval_, 0.1);
    private_nh_.param<double>("position_tolerance", position_tolerance_, 0.02);
    private_nh_.param<double>("yaw_tolerance", yaw_tolerance_, 0.02);

    // Apply namespace
    if (!namespace_param_.empty()) {
        if (namespace_param_[0] != '/') namespace_param_ = "/" + namespace_param_;
        odom_topic_ = namespace_param_ + "/" + odom_topic_;
        marker_topic_ = namespace_param_ + "/" + marker_topic_;
    }

    // Subscribe to odometry
    odom_sub_ = nh_.subscribe(odom_topic_, 100, &TrajectoryPublisherSaver::odomCallback, this);

    // Publish trajectory markers
    trajectory_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 10);

    // Advertise service
    save_service_ = nh_.advertiseService("save_trajectory", &TrajectoryPublisherSaver::saveTrajectory, this);

    ROS_INFO("Trajectory Publisher and Saver started");
}

void TrajectoryPublisherSaver::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_time = ros::Time::now();
    if ((current_time - prev_time).toSec() >= marker_interval_) {
        prev_time = current_time;
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double yaw = quaternionToYaw(msg->pose.pose.orientation);
        double time = ros::Time::now().toSec();

        current_x = x;
        current_y = y;
        current_yaw = yaw;

        if (std::abs(current_x - prev_x) >= position_tolerance_ ||
            std::abs(current_y - prev_y) >= position_tolerance_ ||
            std::abs(current_yaw - prev_yaw) >= yaw_tolerance_) {
            
            trajectory_.push_back({x, y, yaw, time});
            publishMarkers();
            prev_x = current_x;
            prev_y = current_y;
            prev_yaw = current_yaw;
        }
    }
}

void TrajectoryPublisherSaver::publishMarkers() {
    visualization_msgs::MarkerArray marker_array;
    int id = 0;

    for (const auto& point : trajectory_) {
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
        marker.scale.x = 0.1;
        marker.scale.y = 0.02;
        marker.scale.z = 0.04;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration(0);

        marker_array.markers.push_back(marker);
    }

    trajectory_pub_.publish(marker_array);
}

bool TrajectoryPublisherSaver::saveTrajectory(trajectory_visualizer::SaveTrajectory::Request &req, 
                                              trajectory_visualizer::SaveTrajectory::Response &res) {
    double current_time = ros::Time::now().toSec();
    double duration = req.duration;
    std::string filename = req.filename;

    std::vector<TrajectoryPoint> filtered_trajectory;
    for (const auto& point : trajectory_) {
        if (point.time >= current_time - duration) {
            filtered_trajectory.push_back(point);
        }
    }

    if (filtered_trajectory.empty()) {
        res.success = false;
        res.message = "No trajectory data available in the specified time range.";
        return true;
    }

    saveAsCSV(filename, filtered_trajectory);
    res.success = true;
    res.message = "Trajectory saved successfully.";
    return true;
}

void TrajectoryPublisherSaver::saveAsCSV(const std::string& filename, const std::vector<TrajectoryPoint>& data) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", filename.c_str());
        return;
    }

    file << "time,x,y,yaw\n";
    for (const auto& point : data) {
        file << point.time << "," << point.x << "," << point.y << "," << point.yaw << "\n";
    }
    file.close();
    ROS_INFO("Trajectory saved as CSV: %s", filename.c_str());
}

geometry_msgs::Quaternion TrajectoryPublisherSaver::yawToQuaternion(double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);

    geometry_msgs::Quaternion quat_msg;
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();
    quat_msg.w = q.w();

    return quat_msg;
}

double TrajectoryPublisherSaver::quaternionToYaw(const geometry_msgs::Quaternion quat_msg) {
    tf2::Quaternion q(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}
