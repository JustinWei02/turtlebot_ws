cmake_minimum_required(VERSION 3.8)
project(turtlebot3_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)

install(
  DIRECTORY models launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files explicitly
install(
  PROGRAMS
  launch/turtlebot3_gazebo.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export dependencies to ensure proper linking
ament_export_dependencies(urdf ros_gz_bridge ros_gz_sim)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()