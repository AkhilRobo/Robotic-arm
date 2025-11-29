ros2 launch panda_description beer.launch.py

ros2 run my_pcl_processor cloud_tf

ros2 run my_pcl_processor feture_ext_plc_node


ros2 run my_pcl_processor feature_extractor --ros-args \
  -p model_name:=bottle \
  -p max_samples:=5 \
  -p capture_interval_sec:=30.0
