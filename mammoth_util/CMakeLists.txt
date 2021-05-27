cmake_minimum_required(VERSION 3.5)
project(mammoth_util)

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

################################################################################

# FIND AMENT DEPENDENCIES

################################################################################

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

################################################################################

# BUILDING

################################################################################

add_executable(mammoth_util src/costmap_loader.cpp)
ament_target_dependencies(
  mammoth_util
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  OpenCV
)
target_include_directories(mammoth_util PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

################################################################################

# INSTALL

################################################################################

install(
  DIRECTORY launch config data
	DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS mammoth_util
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################

# TESTING

################################################################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
