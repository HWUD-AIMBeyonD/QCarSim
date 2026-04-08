cmake_minimum_required(VERSION 3.11)
project(qcar_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch meshes models rviz urdf
        DESTINATION share/${PROJECT_NAME})

ament_package()
