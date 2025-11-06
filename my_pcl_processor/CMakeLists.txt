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

add_executable(pcl_node src/pcl_node.cpp)
target_link_libraries(pcl_node ${PCL_LIBRARIES})


ament_target_dependencies(pcl_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

install(TARGETS
  pcl_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
