cmake_minimum_required(VERSION 3.0.2)
project(ros_opencv)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

#add
find_package(OpenCV REQUIRED)

catkin_package(
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ros_opencv_pub src/ros_opencv_pub.cpp)
add_dependencies(ros_opencv_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${OpenCV_LIBRARIES})
target_link_libraries(ros_opencv_pub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(ros_opencv_sub src/ros_opencv_sub.cpp)
add_dependencies(ros_opencv_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${OpenCV_LIBRARIES})
target_link_libraries(ros_opencv_sub ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

