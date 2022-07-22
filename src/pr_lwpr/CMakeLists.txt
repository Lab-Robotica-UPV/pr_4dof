cmake_minimum_required(VERSION 3.5)
project(pr_lwpr)

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

####### PONER ESTA LÍNEA SIEMPRE EN LOS CMAKE!!!!!!#########
####### AUMENTA MUCHO LA VELOCIDAD!!!!! ############
set(CMAKE_BUILD_TYPE "Release")

find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/LWPR/lib_ros/lib lwpr)

include_directories(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/LWPR/lib_ros/include)


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

## lwpr_fwd node ###

add_library(lwpr_fwd SHARED
  src/lwpr_fwd.cpp)

target_link_libraries(lwpr_fwd "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/LWPR/lib_ros/lib" lwpr)

target_compile_definitions(lwpr_fwd
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(lwpr_fwd
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib"
  "Eigen3"
  "message_filters")

rclcpp_components_register_nodes(lwpr_fwd "pr_lwpr::LWPRFwd")
set(node_plugins "${node_plugins}pr_lwpr::LWPRFwd;$<TARGET_FILE:lwpr_fwd>\n")

## lwpr_inv node ###

add_library(lwpr_inv SHARED
  src/lwpr_inv.cpp)

target_link_libraries(lwpr_inv "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/LWPR/lib_ros/lib" lwpr)

target_compile_definitions(lwpr_inv
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(lwpr_inv
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib"
  "Eigen3"
  "message_filters")

rclcpp_components_register_nodes(lwpr_inv "pr_lwpr::LWPRInv")
set(node_plugins "${node_plugins}pr_lwpr::LWPRInv;$<TARGET_FILE:lwpr_inv>\n")

### Installation ###

install(TARGETS
  lwpr_fwd
  lwpr_inv
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
