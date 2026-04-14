cmake_minimum_required(VERSION 3.5)
project(stanley_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(stanley_controller SHARED src/stanley_controller.cpp)
target_include_directories(stanley_controller PUBLIC include)
ament_target_dependencies(stanley_controller
  rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d pluginlib
  tf2_ros tf2_geometry_msgs geometry_msgs nav_msgs)

pluginlib_export_plugin_description_file(nav2_core stanley_controller_plugin.xml)

install(TARGETS stanley_controller DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES stanley_controller_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()
