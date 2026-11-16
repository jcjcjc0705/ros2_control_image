cmake_minimum_required(VERSION 3.8)
project(hw_transport)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Core library: CanBus base class, registry, serial helper. Transport plugins and
# hardware components link against it, so the registry is one instance per process.
add_library(hw_transport SHARED
  src/can_bus.cpp
  src/registry.cpp
  src/serial_port.cpp)
target_compile_features(hw_transport PUBLIC cxx_std_17)
target_include_directories(hw_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(hw_transport pluginlib rclcpp)

# Built-in transport plugin: Linux SocketCAN.
add_library(hw_transport_socketcan SHARED src/socketcan.cpp)
target_link_libraries(hw_transport_socketcan hw_transport)
ament_target_dependencies(hw_transport_socketcan pluginlib rclcpp)
pluginlib_export_plugin_description_file(hw_transport hw_transport_plugins.xml)

add_executable(can_tool src/can_tool.cpp)
target_link_libraries(can_tool hw_transport)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS hw_transport EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS hw_transport_socketcan LIBRARY DESTINATION lib)
install(TARGETS can_tool RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(pluginlib rclcpp)
ament_package()
