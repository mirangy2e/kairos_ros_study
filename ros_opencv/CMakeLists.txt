cmake_minimum_required(VERSION 3.0.2)
project(ros_opencv)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  sensor_msgs
  serial
  std_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_opencv
#  CATKIN_DEPENDS cv_bridge image_transport message_generation roscpp sensro_msgs serial stdmsgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
## ros_opencv 프로젝트를 빌드해 만들 실행파일을 선언
## 먼저 ros_opencv_pub 실행파일을 선언  
add_executable(ros_opencv_pub src/ros_opencv_pub.cpp)
target_link_libraries(ros_opencv_pub
    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

## 다음에는 service client 실행파일을 선언, 역시 자동생성 된 텍스트를 원하는 것으로 수정함 
add_executable(ros_opencv_sub src/ros_opencv_sub.cpp)
target_link_libraries(ros_opencv_sub
    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)
