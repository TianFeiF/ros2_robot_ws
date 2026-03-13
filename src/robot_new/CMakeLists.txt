cmake_minimum_required(VERSION 3.8)
project(robot_new)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY launch urdf config meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
