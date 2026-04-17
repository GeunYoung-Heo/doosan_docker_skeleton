cmake_minimum_required(VERSION 3.8)
project(onrobot_rg2_ft)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GripperState.msg"
  "srv/SetGripper.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Install the node executable
install(
  PROGRAMS onrobot_rg2_ft/gripper_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gripper_node
)

# Install compute_box_client.py alongside the node so gripper_node can find it
install(
  FILES onrobot_rg2_ft/compute_box_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
