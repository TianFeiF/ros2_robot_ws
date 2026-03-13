cmake_minimum_required(VERSION 3.8)
project(ethercat_device_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_manager REQUIRED)

add_library(ethercat_device_control SHARED
  src/ethercat_hardware_interface.cpp
)

target_include_directories(ethercat_device_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ethercat_device_control
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ros2_control
  controller_manager
)

# Manually specify EtherCAT library paths
include_directories(/usr/local/etherlab/include)
link_directories(/usr/local/etherlab/lib)

# Link directly to the static library or specific .so file if generic link fails
# or ensure LD_LIBRARY_PATH includes /usr/local/etherlab/lib
target_link_libraries(ethercat_device_control /usr/local/etherlab/lib/libethercat.so.1.2.0)

pluginlib_export_plugin_description_file(hardware_interface ethercat_device_control.xml)

install(
  TARGETS ethercat_device_control
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES ethercat_device_control.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(ethercat_device_control)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ros2_control
  controller_manager
)

ament_package()
