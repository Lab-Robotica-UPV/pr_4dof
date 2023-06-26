cmake_minimum_required(VERSION 3.5)
project(pr_topic_forwarding)

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
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
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


### Topic forwarding node ###

add_library(topic_forwarding SHARED
  src/topic_forwarding.cpp)

target_compile_definitions(topic_forwarding
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(topic_forwarding
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(topic_forwarding "pr_topic_forwarding::TopicForwarding")
set(node_plugins "${node_plugins}pr_topic_forwarding::TopicForwarding;$<TARGET_FILE:topic_forwarding>\n")

### Array to quaternion node ###

add_library(array_to_quaternion SHARED
  src/array_to_quaternion.cpp)

target_compile_definitions(array_to_quaternion
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(array_to_quaternion
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "geometry_msgs"
)

rclcpp_components_register_nodes(array_to_quaternion "pr_topic_forwarding::ArrayToQuaternion")
set(node_plugins "${node_plugins}pr_topic_forwarding::ArrayToQuaternion;$<TARGET_FILE:array_to_quaternion>\n")

### Array to wrench node ###

add_library(array_to_wrench SHARED
  src/array_to_wrench.cpp)

target_compile_definitions(array_to_wrench
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(array_to_wrench
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "geometry_msgs"
)

rclcpp_components_register_nodes(array_to_wrench "pr_topic_forwarding::ArrayToWrench")
set(node_plugins "${node_plugins}pr_topic_forwarding::ArrayToWrench;$<TARGET_FILE:array_to_wrench>\n")

### Force state to wrench node ###

add_library(force_state_to_wrench SHARED
  src/force_state_to_wrench.cpp)

target_compile_definitions(force_state_to_wrench
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(force_state_to_wrench
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "geometry_msgs"
)

rclcpp_components_register_nodes(force_state_to_wrench "pr_topic_forwarding::ForceStateToWrench")
set(node_plugins "${node_plugins}pr_topic_forwarding::ForceStateToWrench;$<TARGET_FILE:force_state_to_wrench>\n")

### Float to float node ###

add_library(float_to_float SHARED
  src/float_to_float.cpp)

target_compile_definitions(float_to_float
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(float_to_float
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "std_msgs"
)

rclcpp_components_register_nodes(float_to_float "pr_topic_forwarding::FloatToFloat")
set(node_plugins "${node_plugins}pr_topic_forwarding::FloatToFloat;$<TARGET_FILE:float_to_float>\n")


install(TARGETS
topic_forwarding
array_to_quaternion
array_to_wrench
force_state_to_wrench
float_to_float

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
