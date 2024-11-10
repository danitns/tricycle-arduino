cmake_minimum_required(VERSION 3.16)
project(tricycle_arduino LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  tricycle_arduino
  SHARED
  hardware/carlikebot_system.cpp
)
target_compile_features(tricycle_arduino PUBLIC cxx_std_17)
target_include_directories(tricycle_arduino PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/tricycle_arduino>
)
ament_target_dependencies(
  tricycle_arduino PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(tricycle_arduino PUBLIC serial)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface tricycle_arduino.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/tricycle_arduino
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/tricycle_arduino
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/tricycle_arduino
)
install(TARGETS tricycle_arduino
  EXPORT export_tricycle_arduino
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_11_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_11_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_11_launch test/test_carlikebot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_tricycle_arduino HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
