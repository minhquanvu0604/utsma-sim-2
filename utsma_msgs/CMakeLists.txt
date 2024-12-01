cmake_minimum_required(VERSION 3.8)
project(utsma_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# List of message files
set(msg_files
  "msg/StateMachineState.msg"
  "msg/VehicleState.msg"
  "msg/VehicleCommand.msg"
  "msg/VehicleCommandStamped.msg"
  "msg/ConeArrayWithCovariance.msg"
  "msg/ConeArrayWithCovariancePlus.msg"
  "msg/ConeWithCovariance.msg"
  "msg/ConeWithCovariancePlus.msg"
)

set(srv_files
  "srv/SendPacket.srv"
)

set(action_files
  "action/ComputeLocalOptimalPath.action"
  "action/FollowLocalPath.action"
)

# Generate messages & services
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs can_msgs geometry_msgs action_msgs nav_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
