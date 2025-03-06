#include "trajectory_visualizer/trajectory_reader_publisher.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_reader_publisher_node");
    TrajectoryReaderPublisher reader;
    ros::spin();
    return 0;
}
