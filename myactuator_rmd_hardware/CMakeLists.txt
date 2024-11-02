cmake_minimum_required(VERSION 3.8)
project(myactuator_rmd_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(myactuator_rmd REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME}
  src/myactuator_rmd_hardware_interface.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC
  cxx_std_17
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${PROJECT_NAME} PRIVATE
  MYACTUATOR_RMD_HARDWARE_BUILDING_LIBRARY
)
# Required for std::atomic
target_link_libraries(${PROJECT_NAME} -latomic)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  myactuator_rmd
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface
  myactuator_rmd_hardware.xml
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)
ament_package()
