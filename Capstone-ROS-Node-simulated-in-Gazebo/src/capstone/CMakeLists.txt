cmake_minimum_required(VERSION 2.8.3)
project(capstone)

find_package(catkin
  REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)
find_package(Boost
  REQUIRED COMPONENTS
  thread
)
find_package(OpenCV REQUIRED)

catkin_package(INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    tf2
    tf2_geometry_msgs
  DEPENDS Boost opencv
)

include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(path_manager
  src/path_manager.cpp
  src/robot_pose_publisher.cpp
)
target_link_libraries(path_manager ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(path_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node path_manager ${catkin_LIBRARIES})

#add_executable(robot_pose_pub_node
#  src/robot_pose_publisher.cpp
#)
#add_dependencies(robot_pose_pub_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(robot_pose_pub_node path_manager ${catkin_LIBRARIES})
