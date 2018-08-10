cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl_visualizer_viewports)



find_package(PCL 1.2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf
)


include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})




add_executable (pcl_visualizer_demo pcl_visualizer_demo.cpp)
target_link_libraries (pcl_visualizer_demo ${catkin_LIBRARIES} ${PCL_LIBRARIES})



add_executable (pcl_ransac pcl_ransac.cpp)
target_link_libraries (pcl_ransac ${catkin_LIBRARIES} ${PCL_LIBRARIES})
