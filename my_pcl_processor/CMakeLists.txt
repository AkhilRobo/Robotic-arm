cmake_minimum_required(VERSION 3.8)
project(my_pcl_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(PCL 1.10 REQUIRED COMPONENTS common filters segmentation kdtree search)

# pcl_node for processing point clouds
add_executable(pcl_node src/pcl_node.cpp)
target_link_libraries(pcl_node ${PCL_LIBRARIES})


ament_target_dependencies(pcl_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

# pcd_saver_node for saving point clouds to PCD files
add_executable(pcd_saver_node src/pcd_saver_node.cpp)
ament_target_dependencies(pcd_saver_node
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(pcd_saver_node ${PCL_LIBRARIES})

install(TARGETS
  pcl_node
  pcd_saver_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
