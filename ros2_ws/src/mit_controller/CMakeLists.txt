cmake_minimum_required(VERSION 3.8)
project(mit_controller)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  controller_interface
  hardware_interface
  mit_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(mit_controller SHARED src/mit_controller.cpp)
target_compile_features(mit_controller PUBLIC cxx_std_17)
target_include_directories(mit_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(mit_controller ${DEPENDENCIES})
pluginlib_export_plugin_description_file(controller_interface mit_controller.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS mit_controller EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES})
ament_package()
