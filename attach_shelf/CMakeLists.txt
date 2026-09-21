cmake_minimum_required(VERSION 3.8)
project(attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# Service dependencies
find_package(custom_interface REQUIRED)  # ← Ajoute ça


add_executable(pre_approach_exe src/pre_approach.cpp)
ament_target_dependencies(pre_approach_exe rclcpp geometry_msgs sensor_msgs rclcpp nav_msgs tf2 tf2_geometry_msgs)

add_executable(pre_approach_v2_exe src/pre_approach_v2.cpp)
ament_target_dependencies(pre_approach_v2_exe rclcpp geometry_msgs sensor_msgs nav_msgs tf2 tf2_geometry_msgs custom_interface)

add_executable(test_service_client_exe src/test_service_client.cpp)
ament_target_dependencies(test_service_client_exe rclcpp custom_interface)

add_executable(approach_service_server_exe src/approach_service_server.cpp)
ament_target_dependencies(approach_service_server_exe rclcpp custom_interface)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  pre_approach_exe
  pre_approach_v2_exe
  test_service_client_exe
  approach_service_server_exe
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz_config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
