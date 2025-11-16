cmake_minimum_required(VERSION 3.8)
project(my_pcl_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)


find_package(PCL 1.10 REQUIRED COMPONENTS common filters segmentation kdtree search)

# ------------_PCL_Node_------------
add_executable(pcl_node src/pcl_node.cpp)
target_link_libraries(pcl_node ${PCL_LIBRARIES})


ament_target_dependencies(pcl_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

# ------------_PCD_Saver_Node_------------
add_executable(pcd_saver_node src/pcd_saver_node.cpp)
ament_target_dependencies(pcd_saver_node
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(pcd_saver_node ${PCL_LIBRARIES})

# ------------_Cloud_Transformer_------------
add_executable(cloud_tf src/cloud_transformer.cpp)
ament_target_dependencies(cloud_tf
  rclcpp
  sensor_msgs
  tf2_ros
  pcl_ros
  geometry_msgs
  
)
target_link_libraries(cloud_tf ${PCL_LIBRARIES})

# ------------_PCL_Node_for_Feture_Extension_------------
add_executable(feture_ext_plc_node src/pcl_node_for_feture_ext.cpp)
ament_target_dependencies(feture_ext_plc_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(feture_ext_plc_node ${PCL_LIBRARIES})


# ----------------feature_extractor------------
add_executable(feature_extractor src/feature_extractor.cpp)
ament_target_dependencies(feature_extractor
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(feature_extractor ${PCL_LIBRARIES})




# ------------_Installation_------------

install(TARGETS
  pcl_node
  pcd_saver_node
  cloud_tf
  feture_ext_plc_node
  feature_extractor
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
