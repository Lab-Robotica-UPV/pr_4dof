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

####### PONER ESTA LÍNEA SIEMPRE EN LOS CMAKE!!!!!!#########
####### AUMENTA MUCHO LA VELOCIDAD!!!!! ############
set(CMAKE_BUILD_TYPE "Release")

#Finds boost components
include(ConfigExtras.cmake)

link_directories ( ${Boost_LIBRARY_DIRS} )
include_directories ( ${Boost_INCLUDE_DIRS} )

find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/lib dmp)
find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/lib dynamicalsystems)
find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/lib functionapproximators)
find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib gmr)
find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib Macros)
find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib Matrix)
find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib Vector)


include_directories(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/include)
include_directories(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/include)
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

### Dmp Evader Force node ###

add_library(dmp_evader_force SHARED
  src/dmp_evader_force.cpp)

target_compile_definitions(dmp_evader_force
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(dmp_evader_force
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(dmp_evader_force "pr_dmp::EvaderForce")
set(node_plugins "${node_plugins}pr_dmp::EvaderForce;$<TARGET_FILE:dmp_evader_force>\n")

### Dmp Evader Force Pid node ###

add_library(dmp_evader_force_pid SHARED
  src/dmp_evader_force_pid.cpp)

target_compile_definitions(dmp_evader_force_pid
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(dmp_evader_force_pid
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(dmp_evader_force_pid "pr_dmp::EvaderForcePID")
set(node_plugins "${node_plugins}pr_dmp::EvaderForcePID;$<TARGET_FILE:dmp_evader_force_pid>\n")

### Dmp Evader Force cart node ###

add_library(dmp_evader_force_cart SHARED
  src/dmp_evader_force_cart.cpp)

target_compile_definitions(dmp_evader_force_cart
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(dmp_evader_force_cart
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(dmp_evader_force_cart "pr_dmp::EvaderForceCart")
set(node_plugins "${node_plugins}pr_dmp::EvaderForceCart;$<TARGET_FILE:dmp_evader_force_cart>\n")

### Dmp_ref_dmp_force node ###

add_library(dmp_ref_force SHARED
  src/dmp_ref_force.cpp)

target_link_libraries(dmp_ref_force ${Boost_LIBRARIES})
target_link_libraries(dmp_ref_force "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/lib" dmp)
target_link_libraries(dmp_ref_force "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/lib" dynamicalsystems)
target_link_libraries(dmp_ref_force "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/lib" functionapproximators)

target_compile_definitions(dmp_ref_force
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(dmp_ref_force
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "Eigen3"
  "pr_lib"
  "Boost"
  )

rclcpp_components_register_nodes(dmp_ref_force "pr_dmp::DmpRefForce")
set(node_plugins "${node_plugins}pr_dmp::DmpRefForce;$<TARGET_FILE:dmp_ref_force>\n")

### Dmp_force_adapted node ###

add_library(dmp_force_adapted SHARED
  src/dmp_force_adapted.cpp)

target_compile_definitions(dmp_force_adapted
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(dmp_force_adapted
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "Eigen3"
)

rclcpp_components_register_nodes(dmp_force_adapted "pr_dmp::ForceAdapted")
set(node_plugins "${node_plugins}pr_dmp::ForceAdapted;$<TARGET_FILE:dmp_force_adapted>\n")

### dmp_rev node ###

add_library(dmp_rev SHARED
  src/dmp_reversible.cpp)

target_link_libraries(dmp_rev ${Boost_LIBRARIES})
target_link_libraries(dmp_rev "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/lib" dmp)
target_link_libraries(dmp_rev "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/lib" dynamicalsystems)
target_link_libraries(dmp_rev "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/DMP/lib_ros/lib" functionapproximators)

target_compile_definitions(dmp_rev
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(dmp_rev
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "Eigen3"
  "pr_lib"
  "Boost")

rclcpp_components_register_nodes(dmp_rev "pr_dmp::DmpRev")
set(node_plugins "${node_plugins}pr_dmp::DmpRev;$<TARGET_FILE:dmp_rev>\n")

### dmp_rev node ###

add_library(dmp_rev_gmr SHARED
  src/dmp_reversible_gmr.cpp)

target_link_libraries(dmp_rev_gmr "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib" gmr)
target_link_libraries(dmp_rev_gmr "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib" Macros)
target_link_libraries(dmp_rev_gmr "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib" Matrix)
target_link_libraries(dmp_rev_gmr "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib" Vector)


target_compile_definitions(dmp_rev_gmr
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(dmp_rev_gmr
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "Eigen3"
  "pr_lib")

rclcpp_components_register_nodes(dmp_rev_gmr "pr_dmp::DmpRevGMR")
set(node_plugins "${node_plugins}pr_dmp::DmpRevGMR;$<TARGET_FILE:dmp_rev_gmr>\n")

### dmp_rev_force_lim node ###

add_library(dmp_rev_flim SHARED
  src/dmp_reversible_force_lim.cpp)

target_link_libraries(dmp_rev_flim "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib" gmr)
target_link_libraries(dmp_rev_flim "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib" Macros)
target_link_libraries(dmp_rev_flim "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib" Matrix)
target_link_libraries(dmp_rev_flim "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib" Vector)


target_compile_definitions(dmp_rev_flim
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(dmp_rev_flim
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "Eigen3"
  "pr_lib")

rclcpp_components_register_nodes(dmp_rev_flim "pr_dmp::DmpRevFLim")
set(node_plugins "${node_plugins}pr_dmp::DmpRevFLim;$<TARGET_FILE:dmp_rev_flim>\n")

# Installation

install(TARGETS
  dmp_ref_gen
  dmp_admittance_force
  dmp_evader_force
  dmp_evader_force_pid
  dmp_ref_force
  dmp_force_adapted
  dmp_evader_force_cart
  dmp_rev
  dmp_rev_gmr
  dmp_rev_flim
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package(CONFIG_EXTRAS ConfigExtras.cmake)
