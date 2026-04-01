cmake_minimum_required(VERSION 3.8)
project(turtlebot4_custom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# buildchain dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

# core functionality dependencies
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)

# message dependencies
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(irobot_create_msgs REQUIRED)

add_library(prompt_behavior src/prompt_behavior.cpp)
add_library(turtlebot4_custom::prompt_behavior ALIAS prompt_behavior)
target_compile_features(prompt_behavior PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(prompt_behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  prompt_behavior
  "rclpy"
  "rclcpp"
  "sensor_msgs"
  "nav2_msgs"
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(prompt_behavior PRIVATE "TURTLEBOT4_CUSTOM_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS prompt_behavior
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Executable Nodes
add_executable(simple_node src/simple_node.cpp)
ament_target_dependencies(
 simple_node
 "rclcpp"
 "irobot_create_msgs"
)

install(TARGETS simple_node
DESTINATION lib/${PROJECT_NAME})

add_executable(command_auto_node src/command_auto_node.cpp)
target_include_directories(command_auto_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(command_auto_node prompt_behavior)

install(TARGETS command_auto_node
  DESTINATION lib/${PROJECT_NAME})

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

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  prompt_behavior
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
