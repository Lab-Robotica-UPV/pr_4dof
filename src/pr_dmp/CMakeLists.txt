cmake_minimum_required(VERSION 3.5)
project(pr_dmp)

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
find_package(pr_lib REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

#Finds boost components
include(ConfigExtras.cmake)

link_directories ( ${Boost_LIBRARY_DIRS} )
include_directories ( ${Boost_INCLUDE_DIRS} )

find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/lib dmp)
find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/lib dynamicalsystems)
find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/lib functionapproximators)

include_directories(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/include)
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

### dmp_ref_gen node ###

add_library(dmp_ref_gen SHARED
  src/dmp_ref_gen.cpp)

target_link_libraries(dmp_ref_gen ${Boost_LIBRARIES})
target_link_libraries(dmp_ref_gen "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/lib" dmp)
target_link_libraries(dmp_ref_gen "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/lib" dynamicalsystems)
target_link_libraries(dmp_ref_gen "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/lib" functionapproximators)

target_compile_definitions(dmp_ref_gen
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(dmp_ref_gen
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "Eigen3"
  "pr_lib"
  "Boost")

rclcpp_components_register_nodes(dmp_ref_gen "pr_dmp::DmpRefGen")
set(node_plugins "${node_plugins}pr_dmp::DmpRefGen;$<TARGET_FILE:dmp_ref_gen>\n")

### Dmp Admittance Model node ###

add_library(dmp_admittance_force SHARED
  src/dmp_admittance_force.cpp)

target_compile_definitions(dmp_admittance_force
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(dmp_admittance_force
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "Eigen3")

rclcpp_components_register_nodes(dmp_admittance_force "pr_dmp::AdmittanceForce")
set(node_plugins "${node_plugins}pr_dmp::AdmittanceForce;$<TARGET_FILE:dmp_admittance_force>\n")

# Installation

install(TARGETS
  dmp_ref_gen
  dmp_admittance_force
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
