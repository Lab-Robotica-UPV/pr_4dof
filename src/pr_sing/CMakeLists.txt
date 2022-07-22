cmake_minimum_required(VERSION 3.5)
project(pr_sing)

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
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)

find_package(pr_msgs REQUIRED)
find_package(pr_lib REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

####### PONER ESTA LÍNEA SIEMPRE EN LOS CMAKE!!!!!!#########
####### AUMENTA MUCHO LA VELOCIDAD!!!!! ############
set(CMAKE_BUILD_TYPE "Release")

include_directories(include)

set(node_plugins "")


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

### Sing Releaser node ###

add_library(sing_releaser SHARED
  src/sing_releaser.cpp)

target_compile_definitions(sing_releaser
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(sing_releaser
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "std_msgs"
  "pr_lib"
  "Eigen3"
  "message_filters")

rclcpp_components_register_nodes(sing_releaser "pr_sing::SingReleaser")
set(node_plugins "${node_plugins}pr_sing::SingReleaser;$<TARGET_FILE:sing_releaser>\n")

### Sing Evader node ###

add_library(sing_evader SHARED
  src/sing_evader.cpp)

target_compile_definitions(sing_evader
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(sing_evader
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "std_msgs"
  "pr_lib"
  "Eigen3"
  "message_filters")

rclcpp_components_register_nodes(sing_evader "pr_sing::SingEvader")
set(node_plugins "${node_plugins}pr_sing::SingEvader;$<TARGET_FILE:sing_evader>\n")

### Installation ###

install(TARGETS
  sing_releaser
  sing_evader
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
