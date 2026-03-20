cmake_minimum_required(VERSION 3.8)
project(rm_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rm_ros_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(rm_hardware_interface SHARED
  src/rm_hardware_interface.cpp
)

target_include_directories(rm_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rm_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  rm_ros_interfaces
  sensor_msgs
  std_msgs
)

pluginlib_export_plugin_description_file(hardware_interface rm_hardware_interface.xml)

install(
  TARGETS rm_hardware_interface
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(rm_hardware_interface)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rm_ros_interfaces
  sensor_msgs
  std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
