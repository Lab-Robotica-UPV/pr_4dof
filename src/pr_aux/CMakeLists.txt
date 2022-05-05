cmake_minimum_required(VERSION 3.5)
project(pr_aux)

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
find_package(message_filters REQUIRED)

find_package(pr_msgs REQUIRED)
find_package(pr_lib REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

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

### Derivator node ###

add_library(derivator SHARED
  src/derivator.cpp)

target_compile_definitions(derivator
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(derivator
  "rclcpp"
  "rclcpp_components"
  "pr_msgs")

rclcpp_components_register_nodes(derivator "pr_aux::Derivator")
set(node_plugins "${node_plugins}pr_aux::Derivator;$<TARGET_FILE:derivator>\n")

### Replayer node ###

add_library(replayer SHARED
  src/replayer.cpp)

target_compile_definitions(replayer
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(replayer
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib")

rclcpp_components_register_nodes(replayer "pr_aux::Replayer")
set(node_plugins "${node_plugins}pr_aux::Replayer;$<TARGET_FILE:replayer>\n")

### Replayer pos force node ###

add_library(replayer_pos_force SHARED
  src/replayer_pos_force.cpp)

target_compile_definitions(replayer_pos_force
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(replayer_pos_force
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib")

rclcpp_components_register_nodes(replayer_pos_force "pr_aux::ReplayerPosForce")
set(node_plugins "${node_plugins}pr_aux::ReplayerPosForce;$<TARGET_FILE:replayer_pos_force>\n")

### AddGain node ###

add_library(add_gain SHARED
  src/add_gain.cpp)

target_compile_definitions(add_gain
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(add_gain
  "rclcpp"
  "rclcpp_components"
  "message_filters"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(add_gain "pr_aux::AddGain")
set(node_plugins "${node_plugins}pr_aux::AddGain;$<TARGET_FILE:add_gain>\n")

### Kalman Filter node ###

add_library(kalman_filter SHARED
  src/kalman_filter.cpp)

target_compile_definitions(kalman_filter
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(kalman_filter
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(kalman_filter "pr_aux::KalmanFilter")
set(node_plugins "${node_plugins}pr_aux::KalmanFilter;$<TARGET_FILE:kalman_filter>\n")

### Displayer node ###

add_library(displayer SHARED
  src/displayer.cpp)

target_compile_definitions(displayer
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(displayer
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(displayer "pr_aux::Displayer")
set(node_plugins "${node_plugins}pr_aux::Displayer;$<TARGET_FILE:displayer>\n")

### Saturator node ###

add_library(saturator SHARED
  src/saturator.cpp)

target_compile_definitions(saturator
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(saturator
  "rclcpp"
  "rclcpp_components"
  "pr_msgs")

rclcpp_components_register_nodes(saturator "pr_aux::Saturator")
set(node_plugins "${node_plugins}pr_aux::Saturator;$<TARGET_FILE:saturator>\n")

### Matrix_mult node ###

add_library(matrix_mult SHARED
  src/matrix_mult.cpp)

target_compile_definitions(matrix_mult
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(matrix_mult
  "rclcpp"
  "rclcpp_components"
  "pr_msgs")

rclcpp_components_register_nodes(matrix_mult "pr_aux::MatrixMult")
set(node_plugins "${node_plugins}pr_aux::MatrixMult;$<TARGET_FILE:MatrixMult>\n")

### Element_wise_mult node ###

add_library(element_wise_mult SHARED
  src/element_wise_mult.cpp)

target_compile_definitions(element_wise_mult
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(element_wise_mult
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib")

rclcpp_components_register_nodes(element_wise_mult "pr_aux::ElementWiseMult")
set(node_plugins "${node_plugins}pr_aux::ElementWiseMult;$<TARGET_FILE:ElementWiseMult>\n")


### Installation ###

install(TARGETS
  derivator
  replayer
  replayer_pos_force
  add_gain
  kalman_filter
  displayer
  saturator
  matrix_mult
  element_wise_mult
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


ament_package()
