cmake_minimum_required(VERSION 3.5)
project(pr_biomech)

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
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pr_lib_biomech REQUIRED)
find_package(pr_lib REQUIRED)

####### PONER ESTA LÍNEA SIEMPRE EN LOS CMAKE!!!!!!#########
####### AUMENTA MUCHO LA VELOCIDAD!!!!! ############
set(CMAKE_BUILD_TYPE "Release")

find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/NatNet/lib_ros/lib NatNet)

include_directories(include)
include_directories(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/rapidjson/lib_ros/include)
include_directories(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/NatNet/lib_ros/include)



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

### streaming_gdlf node ###

add_library(streaming_gdlf SHARED
  src/streaming_gdlf.cpp
  src/Cinematica.cpp
  src/Dinamica.cpp
  src/PelCalcs.cpp
  src/CalTheta2Theta3.cpp
  )

  target_link_libraries(streaming_gdlf "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/NatNet/lib_ros/lib" NatNet)

target_compile_definitions(streaming_gdlf
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(streaming_gdlf
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib_biomech"
  "Eigen3"
)

rclcpp_components_register_nodes(streaming_gdlf "pr_biomech::StreamingGDLF")
set(node_plugins "${node_plugins}pr_biomech::StreamingGDLF;$<TARGET_FILE:streaming_gdlf>\n")

### Algorithm assistance node ###

add_library(algorithm_assistance SHARED
  src/algorithm_assistance.cpp)

target_compile_definitions(algorithm_assistance
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(algorithm_assistance
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(algorithm_assistance "pr_biomech::AlgorithmAssistance")
set(node_plugins "${node_plugins}pr_biomech::AlgorithmAssistance;$<TARGET_FILE:algorithm_assistance>\n")

### Admittance Euler biomech node ###

add_library(admittance_euler_biomech SHARED
  src/admittance_euler_biomech.cpp)

target_compile_definitions(admittance_euler_biomech
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(admittance_euler_biomech
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(admittance_euler_biomech "pr_biomech::AdmittanceEulerBiomech")
set(node_plugins "${node_plugins}pr_biomech::AdmittanceEulerBiomech;$<TARGET_FILE:admittance_euler_biomech>\n")

### Model Pau sim node ###

add_library(model_pau_sim SHARED
  src/model_pau_sim.cpp)

target_compile_definitions(model_pau_sim
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(model_pau_sim
  "rclcpp"
  "rclcpp_components"
  "pr_msgs"
  "pr_lib"
  "Eigen3")

rclcpp_components_register_nodes(model_pau_sim "pr_biomech::ModelPauSim")
set(node_plugins "${node_plugins}pr_biomech::ModelPauSim;$<TARGET_FILE:model_pau_sim>\n")

install(TARGETS
streaming_gdlf
algorithm_assistance
admittance_euler_biomech
model_pau_sim
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