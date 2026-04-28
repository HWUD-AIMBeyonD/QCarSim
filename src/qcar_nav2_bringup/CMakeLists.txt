cmake_minimum_required(VERSION 3.11)
project(qcar_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/save_map.sh
        DESTINATION lib/${PROJECT_NAME})

ament_package()
