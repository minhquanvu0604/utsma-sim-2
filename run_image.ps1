# PowerShell script version

# Make sure you put this file and the img in the same directory
# docker image load -i utsma_demo_img

param(
    [Parameter(Mandatory=$true)]
    [string]$BindMountPath
)

# 1. Run XLaunch
# 2. Run container
docker run  --detach `
            --name ros_humble_pfms_container `
            --gpus all `
            -v "${BindMountPath}:/root/ros2_ws/src" `
            -e DISPLAY=host.docker.internal:0.0 `
            --workdir /root/ros2_ws `
            ros-humble-pfms:latest



# # Run the Docker container for simulation
# docker run --privileged `
#            --detach `
#            --name utsma_demo_sim `
#            --network "host" `
#            -e ROS_MASTER_URI=http://localhost:11311 `
#            -e DISPLAY=host.docker.internal:0.0 `
#            ros-humble-pfms:latest `
#            bash -c "source /opt/ros/noetic/setup.bash && source /utsma_ws/devel/setup.bash && roslaunch utsma_gazebo simulation.launch"

# # Run the Docker container for path planning
# docker run --privileged `
#            --detach `
#            --name utsma_demo_path_planning `
#            --network "host" `
#            -e ROS_MASTER_URI=http://localhost:11311 `
#            -e DISPLAY=host.docker.internal:0.0 `
#            ros-humble-pfms:latest `
#            bash -c "source /opt/ros/noetic/setup.bash && source /utsma_ws/devel/setup.bash && sleep 10 && rosrun d_triang utsma_path_planner"

# docker run --privileged `
#            --detach `
#            --name utsma_demo_code_viewer `
#            --network "host" `
#            -e ROS_MASTER_URI=http://localhost:11311 `
#            -e DISPLAY=host.docker.internal:0.0 `
#            ros-humble-pfms:latest `
#            bash -c "sleep infinity"