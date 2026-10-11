cmake_minimum_required(VERSION 3.11)
project(qcar_nav2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps
        DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/save_map.sh
        DESTINATION lib/${PROJECT_NAME})

# Verbatim copy of the hardware node (see scripts/check_hw_parity.sh)
install(PROGRAMS qcar_nav2_bringup/direct_map_publisher.py
        DESTINATION lib/${PROJECT_NAME}
        RENAME direct_map_publisher)

ament_package()
