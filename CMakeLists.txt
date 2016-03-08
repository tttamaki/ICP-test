cmake_minimum_required(VERSION 2.8)
set(CMAKE_VERBOSE_MAKEFILE 1) # to see all make process

project( icp_samples )

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (icp1_simple src/icp1_simple.cpp)
target_link_libraries (icp1_simple ${PCL_LIBRARIES})

add_executable (icp2_iterative_view src/icp2_iterative_view.cpp)
target_link_libraries (icp2_iterative_view ${PCL_LIBRARIES})

add_executable (icp3_with_normal_iterative_view src/icp3_with_normal_iterative_view.cpp)
target_link_libraries (icp3_with_normal_iterative_view ${PCL_LIBRARIES})

add_executable (icp4_after_feature_registration src/icp4_after_feature_registration.cpp src/visualize_correspondences.cpp)
target_link_libraries (icp4_after_feature_registration ${PCL_LIBRARIES})


add_executable (transform_estimation src/transform_estimation.cpp)
target_link_libraries (transform_estimation ${PCL_LIBRARIES})

