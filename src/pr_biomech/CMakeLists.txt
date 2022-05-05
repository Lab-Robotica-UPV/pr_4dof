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
  src/PelCalcs.cpp
  src/CalConst_Pau2.cpp
  src/CalTheta2Theta3.cpp
  src/DynamicsHerz_Pau.cpp
  src/MusForceOpt2_Anta_Coef_Guard_Red.cpp
  src/CalLigForceAnta_Pau.cpp
  src/FOpt.cpp)

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
set(node_plugins "${node_plugins}pr_biomech::StreamingGDLF;$<TARGET_FILE:pr_biomech>\n")

install(TARGETS
streaming_gdlf
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