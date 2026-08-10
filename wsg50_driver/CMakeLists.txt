cmake_minimum_required(VERSION 3.8)
project(wsg50_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(wsg50_msgs REQUIRED)

add_library(wsg50_transport SHARED
  src/transport.cpp
)
target_include_directories(wsg50_transport PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_library(wsg50_gripper_driver_component SHARED src/main.cpp)
target_compile_definitions(wsg50_gripper_driver_component PRIVATE WSG50_DRIVER_BUILDING_DLL)
target_include_directories(wsg50_gripper_driver_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(wsg50_gripper_driver_component wsg50_transport)
ament_target_dependencies(wsg50_gripper_driver_component
  control_msgs
  diagnostic_msgs
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  std_srvs
  wsg50_msgs
)

rclcpp_components_register_node(wsg50_gripper_driver_component
  PLUGIN "wsg50_driver::GripperDriver"
  EXECUTABLE wsg50_gripper_driver_node
)

install(TARGETS wsg50_transport wsg50_gripper_driver_component
  EXPORT export_wsg50_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_wsg50_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(
  control_msgs diagnostic_msgs rclcpp rclcpp_action rclcpp_components
  sensor_msgs std_srvs wsg50_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_transport test/test_transport.cpp)
  if(TARGET test_transport)
    target_link_libraries(test_transport wsg50_gripper_driver_component wsg50_transport)
    ament_target_dependencies(test_transport
      control_msgs
      rclcpp
      rclcpp_action
      sensor_msgs
      wsg50_msgs
    )
  endif()
endif()

ament_package()
