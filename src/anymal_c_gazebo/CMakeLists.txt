cmake_minimum_required(VERSION 3.5)
project(anymal_c_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)

ament_package()
