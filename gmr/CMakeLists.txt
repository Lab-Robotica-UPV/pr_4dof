cmake_minimum_required(VERSION 3.10)

# set the project name
project(Tutorial)

# add the executable
add_executable(Main main.cpp)
# add_executable(ReadGMR read_gmr.cpp)

# find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib gmr)
# find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib Macros)
# find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib Matrix)
# find_library(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib Vector)

####### PONER ESTA LÍNEA SIEMPRE EN LOS CMAKE!!!!!!#########
####### AUMENTA MUCHO LA VELOCIDAD!!!!! ############
set(CMAKE_BUILD_TYPE "Release")

include_directories(/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/include)

target_link_libraries(Main "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib" gmr)
target_link_libraries(Main "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib" Macros)
target_link_libraries(Main "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib" Matrix)
target_link_libraries(Main "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib" Vector)
# target_link_libraries(ReadGMR "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib" gmr)
# target_link_libraries(ReadGMR "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib" Macros)
# target_link_libraries(ReadGMR "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib" Matrix)
# target_link_libraries(ReadGMR "-L/home/paralelo4dofnew/ros2_eloquent_ws/libs_thirdparty/GMR/lib_ros/lib" Vector)
