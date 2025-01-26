cmake_minimum_required(VERSION 3.8)
project(omnibot_platform_teleop)


# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(include)

pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
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

install(
  DIRECTORY
  config
  include
  src
  launch
  ${YAML_CPP_INCLUDE_DIRS}

  DESTINATION share/${PROJECT_NAME}/
)
add_executable(omnibot_platform_teleop 
  src/omnibot_platform_teleop_node.cpp 
  )
ament_target_dependencies(omnibot_platform_teleop rclcpp std_msgs geometry_msgs sensor_msgs)

install(TARGETS 
omnibot_platform_teleop
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  launch/omnibot_platform_teleop.launch.py
  DESTINATION lib/${PROJECT_NAME})


target_link_libraries(omnibot_platform_teleop
  ${YAML_CPP_LIBRARIES}
)

ament_package()
