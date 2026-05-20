cmake_minimum_required(VERSION 3.5)
project(pcd_to_gridmap)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_to_gridmap src/pcd_to_gridmap.cpp)
target_link_libraries(pcd_to_gridmap ${PCL_LIBRARIES})