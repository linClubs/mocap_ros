cmake_minimum_required(VERSION 3.5)
project(mocap_pkg)
add_definitions(-w)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

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

include_directories(
  include
  ${PROJECT_SOURCE_DIR}/third_party/eigen3.4.0/include
)

add_executable(mocap_test src/mocap_test.cpp)
ament_target_dependencies(mocap_test rclcpp)



add_executable(mocap_ros src/mocap_ros.cpp)
ament_target_dependencies(mocap_ros rclcpp)

find_package(unitree_go REQUIRED)
add_executable(h1_arm_hand src/h1_arm_hand.cpp)
ament_target_dependencies(h1_arm_hand rclcpp unitree_go)

add_executable(mocap_control src/mocap_control.cpp)
ament_target_dependencies(mocap_control rclcpp unitree_go)

install(
  TARGETS
  h1_arm_hand
  mocap_control
  DESTINATION lib/${PROJECT_NAME}
)