cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(whiskeye_plugin VERSION 2.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(gz-math7 REQUIRED)
find_package(gz-physics6 REQUIRED)
find_package(gz-sensors7 REQUIRED)
find_package(gz-sim7 REQUIRED)
find_package(gz-utils2 REQUIRED)

add_library(
  whiskeye_plugin
  src/whiskeye_plugin.cpp)

target_compile_features(
  whiskeye_plugin
  PUBLIC
  c_std_99
  cxx_std_17
)

target_compile_definitions(
  whiskeye_plugin
  PRIVATE
  "WHISKEYE_PLUGIN_BUILDING_LIBRARY"
)

target_include_directories(
  whiskeye_plugin
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/rosidl_generator_cpp>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  whiskeye_plugin
  gz-math7
  gz-physics6
  gz-sensors7
  gz-sim7
  gz-utils2
  pluginlib
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

rosidl_generate_interfaces(
  whiskeye_msgs
  "msg/BodyToBrain.msg"
  "msg/BrainToBody.msg"
  DEPENDENCIES
  builtin_interfaces
  std_msgs
  sensor_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target
  whiskeye_msgs
  "rosidl_typesupport_cpp"
)

target_link_libraries(
  whiskeye_plugin
  "${cpp_typesupport_target}"
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  TARGETS whiskeye_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(whiskeye_plugin)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
