cmake_minimum_required(VERSION 3.5.1)
project(anymal_c_simple_description)

find_package(ament_cmake)


install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
