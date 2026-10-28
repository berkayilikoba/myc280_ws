cmake_minimum_required(VERSION 3.5)
project(limo_bringup)

find_package(ament_cmake REQUIRED)

install(
    DIRECTORY param maps launch behavior_trees
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()