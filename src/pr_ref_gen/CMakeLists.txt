cmake_minimum_required(VERSION 3.5)
project(pr_ref_gen)

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


### Pose Ref Generator node ###

add_library(ref_pose SHARED
  src/ref_pose.cpp)

target_compile_definitions(ref_pose
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(ref_pose
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "std_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(ref_pose "pr_ref_gen::RefPose")
set(node_plugins "${node_plugins}pr_ref_gen::RefPose;$<TARGET_FILE:ref_pose>\n")

### Ref Server node ###

add_library(ref_server SHARED
  src/ref_server.cpp)

target_compile_definitions(ref_server
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(ref_server
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "std_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(ref_server "pr_ref_gen::RefServer")
set(node_plugins "${node_plugins}pr_ref_gen::RefServer;$<TARGET_FILE:ref_server>\n")

### Ref Init Auto Generator node ###

add_library(ref_init_auto SHARED
  src/ref_init_auto.cpp)

target_compile_definitions(ref_init_auto
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(ref_init_auto
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "std_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(ref_init_auto "pr_ref_gen::RefInitAuto")
set(node_plugins "${node_plugins}pr_ref_gen::RefInitAuto;$<TARGET_FILE:ref_init_auto>\n")

### Installation ###

install(TARGETS
  ref_pose
  ref_server
  ref_init_auto
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


ament_package()
