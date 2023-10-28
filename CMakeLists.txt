cmake_minimum_required(VERSION 3.8)
project(ar3_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  ar3_hardware_interface
  SHARED
  src/ar3_hardware_interface.cpp
  src/checksum.cpp
  src/messaging.cpp
  src/serialize.cpp
)
target_link_libraries(
  ar3_hardware_interface
  serial
)
target_compile_features(
  ar3_hardware_interface 
  PUBLIC 
  cxx_std_17
)
target_include_directories(
  ar3_hardware_interface
  PUBLIC
  include
)
ament_target_dependencies(
  ar3_hardware_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)

# Prevent pluginlib from using boost (maybe unecessary)
target_compile_definitions(
  ar3_hardware_interface
  PUBLIC 
  "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
)


pluginlib_export_plugin_description_file(
  hardware_interface ar3_hardware_interface.xml
)

install(
  TARGETS
  ar3_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_export_include_directories(
  include
)
ament_export_libraries(
  ar3_hardware_interface
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()