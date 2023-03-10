cmake_minimum_required(VERSION 3.5)
project(pr_mocap)

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
find_package(pr_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pr_lib REQUIRED)

####### PONER ESTA LÍNEA SIEMPRE EN LOS CMAKE!!!!!!#########
####### AUMENTA MUCHO LA VELOCIDAD!!!!! ############
set(CMAKE_BUILD_TYPE "Release")

find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/NatNet/lib_ros/lib NatNet)

include_directories(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/NatNet/lib_ros/include)

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

### Mocap node ###

add_library(pr_x_mocap SHARED
  src/pr_x_mocap.cpp)

target_link_libraries(pr_x_mocap "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/NatNet/lib_ros/lib" NatNet)

target_compile_definitions(pr_x_mocap
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(pr_x_mocap
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "std_msgs"
  "pr_lib"
  "Eigen3"
)

rclcpp_components_register_nodes(pr_x_mocap "pr_mocap::PRXMocap")
set(node_plugins "${node_plugins}pr_mocap::PRXMocap;$<TARGET_FILE:pr_x_mocap>\n")

### Mocap synchronizer node ###

add_library(pr_x_mocap_synchronizer SHARED
  src/pr_x_mocap_synchronizer.cpp)

target_compile_definitions(pr_x_mocap_synchronizer
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(pr_x_mocap_synchronizer
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
)

rclcpp_components_register_nodes(pr_x_mocap_synchronizer "pr_mocap::PRXMocapSynchronizer")
set(node_plugins "${node_plugins}pr_mocap::PRXMocapSynchronizer;$<TARGET_FILE:pr_x_mocap_synchronizer>\n")

### Mocap recorder node ###

add_library(pr_x_mocap_recorder SHARED
  src/pr_x_mocap_recorder.cpp)

target_compile_definitions(pr_x_mocap_recorder
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(pr_x_mocap_recorder
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "Eigen3"
)

rclcpp_components_register_nodes(pr_x_mocap_recorder "pr_mocap::PRXMocapRecorder")
set(node_plugins "${node_plugins}pr_mocap::PRXMocapRecorder;$<TARGET_FILE:pr_x_mocap_recorder>\n")

install(TARGETS
  pr_x_mocap
  pr_x_mocap_synchronizer
  pr_x_mocap_recorder
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

ament_package()
