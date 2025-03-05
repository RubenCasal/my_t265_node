cmake_minimum_required(VERSION 3.8)
project(my_t265_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realsense2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

add_executable(t265_node src/t265_node.cpp)

target_include_directories(t265_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${realsense2_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)


ament_target_dependencies(t265_node 
  rclcpp 
  realsense2 
  nav_msgs 
  sensor_msgs 
  tf2 
  tf2_ros 
  tf2_msgs 
  tf2_geometry_msgs 
  tf2_sensor_msgs 
  cv_bridge
)


target_link_libraries(t265_node 
  ${OpenCV_LIBS} 
  ${cv_bridge_LIBRARIES}
  ${realsense2_LIBRARY}
)

install(TARGETS
  t265_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
