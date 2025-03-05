#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <vector>
#include "trajectory_visualizer/SaveTrajectory.h"

struct TrajectoryPoint {
    double x, y, time;
};

class TrajectoryPubSave {
private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher trajectory_pub_;
    ros::ServiceServer save_service_;

    std::vector<TrajectoryPoint> trajectory_;
    std::string namespace_param, odom_frame_, odom_topic_, marker_topic_;

public:
    TrajectoryPubSave() {

        // Get parameters
        nh_.param<std::string>("namespace", namespace_param, "");
        nh_.param<std::string>("odom_frame", odom_frame_, "odom");
        nh_.param<std::string>("odom_topic", odom_topic_, "odom");
        nh_.param<std::string>("marker_topic", marker_topic_, "trajectory_markers");


        // Apply namespace if set
        if (!namespace_param.empty()) {
            if (namespace_param[0] != '/') namespace_param = "/" + namespace_param;
            odom_topic_ = namespace_param + "/" + odom_topic_;
            marker_topic_ = namespace_param + "/" + marker_topic_;
        }

        // Subscribe to odometry
        odom_sub_ = nh_.subscribe(odom_topic_, 100, &TrajectoryPubSave::odomCallback, this);

        // Publish trajectory markers
        trajectory_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 10);
        
        // Advertise service
        save_service_ = nh_.advertiseService("save_trajectory", &TrajectoryPubSave::saveTrajectory, this);

        ROS_INFO("Trajectory Manager Initialized. Using namespace: [%s]", namespace_param.c_str());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double time = ros::Time::now().toSec();

        trajectory_.push_back({x, y, time});
        publishMarkers();
    }

    void publishMarkers() {
        visualization_msgs::MarkerArray marker_array;
        int id = 0;

        for (const auto& point : trajectory_) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = odom_frame_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "trajectory";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = point.x;
            marker.pose.position.y = point.y;
            marker.pose.position.z = 0.0;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;
            marker.lifetime = ros::Duration(0);
            
            marker_array.markers.push_back(marker);
        }

        trajectory_pub_.publish(marker_array);
    }

    bool saveTrajectory(trajectory_visualizer::SaveTrajectory::Request &req, 
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

        // Always save as CSV
        saveAsCSV(filename, filtered_trajectory);

        res.success = true;
        res.message = "Trajectory saved successfully.";
        return true;
    }

    void saveAsCSV(const std::string& filename, const std::vector<TrajectoryPoint>& data) {
        std::ofstream file(filename);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            return;
        }

        file << "time,x,y\n";
        for (const auto& point : data) {
            file << point.time << "," << point.x << "," << point.y << "\n";
        }
        file.close();
        ROS_INFO("Trajectory saved as CSV: %s", filename.c_str());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_publisher_saver_node");
    TrajectoryPubSave manager;
    ros::spin();
    return 0;
}
