cmake_minimum_required(VERSION 2.8.3)
project(husky_ur3_gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslaunch
)
find_package(gazebo REQUIRED)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
# 노드 추가 - 210927 YSW
add_executable(gazebo_rh_pub src/gazebo_rh_pub.cpp)
add_dependencies(gazebo_rh_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_rh_pub ${catkin_LIBRARIES})

add_library(animated_box SHARED src/animated_box.cc)
target_link_libraries(animated_box ${GAZEBO_LIBRARIES})

install(TARGETS gazebo_rh_pub
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch controller DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
