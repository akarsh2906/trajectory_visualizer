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
  CATKIN_DEPENDS roscpp std_msgs message_runtime 
)



add_executable(trajectory_pub_save src/trajectory_pub_save.cpp)
add_executable(trajectory_read_pub src/trajectory_read_pub.cpp)


add_dependencies(trajectory_pub_save ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(trajectory_pub_save
  ${catkin_LIBRARIES}
)
target_link_libraries(trajectory_read_pub
  ${catkin_LIBRARIES}
)



install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS})

