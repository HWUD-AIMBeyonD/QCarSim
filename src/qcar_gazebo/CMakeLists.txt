cmake_minimum_required(VERSION 3.11)
project(qcar_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY worlds/
        DESTINATION share/${PROJECT_NAME}/worlds)

install(PROGRAMS scripts/qcar_hw_emulator.py scripts/generate_world_from_map.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
