cmake_minimum_required(VERSION 3.0.2)
project(trajectory_visualizer)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_msgs
  visualization_msgs
  message_generation
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs


)


find_package(catkin REQUIRED COMPONENTS roscpp std_msgs nav_msgs visualization_msgs message_generation geometry_msgs tf2_ros tf2_geometry_msgs)

add_service_files(
  FILES
  SaveTrajectory.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_publisher_saver
  CATKIN_DEPENDS roscpp std_msgs message_runtime nav_msgs visualization_msgs message_generation geometry_msgs tf2_ros tf2_geometry_msgs
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)




add_library(trajectory_publisher_saver
    src/trajectory_publisher_saver.cpp
)

target_link_libraries(trajectory_publisher_saver
    ${catkin_LIBRARIES}
)

add_executable(trajectory_publisher_saver_node src/trajectory_publisher_saver_node.cpp)
target_link_libraries(trajectory_publisher_saver_node
    trajectory_publisher_saver
    ${catkin_LIBRARIES}
)





add_library(trajectory_reader_publisher
    src/trajectory_reader_publisher.cpp
)

target_link_libraries(trajectory_reader_publisher 
    ${catkin_LIBRARIES}
)

add_executable(trajectory_reader_publisher_node src/trajectory_reader_publisher_node.cpp)
target_link_libraries(trajectory_reader_publisher_node
    trajectory_reader_publisher
    ${catkin_LIBRARIES}
)




install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


