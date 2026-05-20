cmake_minimum_required(VERSION 3.5)
project(pcd_to_octomap)

find_package(PCL REQUIRED)
find_package(octomap REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_to_octomap src/pcd_to_octomap.cpp)
target_link_libraries(pcd_to_octomap ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
