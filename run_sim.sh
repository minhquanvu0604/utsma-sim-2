#!/bin/bash

# Give xhost permission to display GUI
xhost +local:*

docker run --privileged \
           --detach \
           --name utsma_sim_2 \
           --network "host" \
           -e DISPLAY=$DISPLAY \
           -v /tmp/.X11-unix:/tmp/.X11-unix \
           --memory=10g \
           utsma-sim-2:latest \
            bash -c "source /opt/ros/humble/setup.bash && source /root/utsma_sim_ws/install/setup.bash && \
           ros2 launch eufs_launcher eufs_launcher.launch.py"

           #    bash -c "source /opt/ros/humble/setup.bash && source /root/utsma_sim_ws/install/setup.bash && \
        #    ros2 launch eufs_launcher simulation.launch.py track:=small_track vehicleModel:=DynamicBicycle 
        #    commandMode:=acceleration vehicleModelConfig:=configDry.yaml robot_name:=eufs rviz:=true 
        #    launch_group:=no_perception gazebo_gui:=true publish_gt_tf:=false pub_ground_truth:=true"