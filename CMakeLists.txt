cmake_minimum_required(VERSION 2.8)

project(iterative_closest_point)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (iterative_closest_point src/iterative_closest_point.cpp)
target_link_libraries (iterative_closest_point ${PCL_LIBRARIES})

add_executable (icp_view src/icp_view.cpp)
target_link_libraries (icp_view ${PCL_LIBRARIES})

