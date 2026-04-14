cmake_minimum_required(VERSION 3.11)
project(qcar_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY worlds/
        DESTINATION share/${PROJECT_NAME}/worlds)

ament_package()
