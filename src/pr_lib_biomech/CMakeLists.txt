cmake_minimum_required(VERSION 3.5)
project(pr_lib_biomech)

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

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/NatNet/lib_ros/lib NatNet)

include_directories(include)
include_directories(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/rapidjson/lib_ros/include)
include_directories(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/NatNet/lib_ros/include)

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

### LIBRARY ###

add_library(pr_lib_biomech SHARED
  src/pr_algebra_fun.cpp
  src/pr_data_structures.cpp
  src/pr_json_calibration.cpp
  src/pr_json_gdlf.cpp
  src/pr_mocap.cpp
  src/pr_biomech_fun.cpp)

target_link_libraries(pr_lib_biomech "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/NatNet/lib_ros/lib" NatNet)


ament_target_dependencies(pr_lib_biomech
  "Eigen3")

target_include_directories(pr_lib_biomech
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_include_directories(include)
ament_export_libraries(pr_lib_biomech)


# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(
  DIRECTORY include/
  DESTINATION include
)

### Installation ###

install(TARGETS
  pr_lib_biomech
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()