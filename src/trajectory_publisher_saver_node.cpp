#include "trajectory_visualizer/trajectory_publisher_saver.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_publisher_saver_node");
    TrajectoryPublisherSaver manager;
    ros::spin();
    return 0;
}
