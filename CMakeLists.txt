cmake_minimum_required(VERSION 3.8)
project(vision_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yolo_msgs REQUIRED)

include_directories(include)


add_executable(tracker_node src/tracker.cpp)
ament_target_dependencies(tracker_node rclcpp yolo_msgs)
install(TARGETS tracker_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
