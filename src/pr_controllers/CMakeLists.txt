cmake_minimum_required(VERSION 3.5)
project(pr_controllers)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
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


### Gus Controller node ###

add_library(gus_controller SHARED
  src/gus_controller.cpp)

target_compile_definitions(gus_controller
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(gus_controller
  "rclcpp"
  "rclcpp_components"
  "message_filters"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(gus_controller "pr_controllers::GusController")
set(node_plugins "${node_plugins}pr_controllers::GusController;$<TARGET_FILE:gus_controller>\n")


### PDG Controller node ###

add_library(pdg_controller SHARED
  src/pdg_controller.cpp)

target_compile_definitions(pdg_controller
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(pdg_controller
  "rclcpp"
  "rclcpp_components"
  "message_filters"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(pdg_controller "pr_controllers::PDGController")
set(node_plugins "${node_plugins}pr_controllers::PDGController;$<TARGET_FILE:pdg_controller>\n")

### PID Controller node ###

add_library(pid_controller SHARED
  src/pid_controller.cpp)

target_compile_definitions(pid_controller
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(pid_controller
  "rclcpp"
  "rclcpp_components"
  "message_filters"
  "pr_msgs"
  "pr_lib")

rclcpp_components_register_nodes(pid_controller "pr_controllers::PIDController")
set(node_plugins "${node_plugins}pr_controllers::PIDController;$<TARGET_FILE:pid_controller>\n")


### Gus Controller node ###

add_library(gus_discretized_controller SHARED
  src/gus_discretized_controller.cpp)

target_compile_definitions(gus_discretized_controller
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(gus_discretized_controller
  "rclcpp"
  "rclcpp_components"
  "message_filters"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(gus_discretized_controller "pr_controllers::GusDiscretizedController")
set(node_plugins "${node_plugins}pr_controllers::GusDiscretizedController;$<TARGET_FILE:gus_discretized_controller>\n")

install(TARGETS
  gus_controller
  pdg_controller
  pid_controller
  gus_discretized_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
