ros2 launch eufs_launcher simulation.launch.py track:=LAST_LAUNCH vehicleModel:=DynamicBicycle commandMode:=acceleration vehicleModelConfig:=configDry.yaml robot_name:=eufs rviz:=true launch_group:=no_perception gazebo_gui:=false publish_gt_tf:=false pub_ground_truth:=true

ros2 launch eufs_launcher simulation.launch.py vehicleModelConfig:=configDry.yaml