cmake_minimum_required(VERSION 3.8)
project(whiteboard_robot)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros2_control REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)

# 1. Install Configuration Files (Launch, URDF, World)
install(
  DIRECTORY urdf launch world config
  DESTINATION share/${PROJECT_NAME}
)

# 2. Install the Python Script as an Executable
# This tells ROS to take your script, name it 'cleaner_node', and put it where 'ros2 run' can find it.
install(PROGRAMS
  whiteboard_robot/whiteboard_cleaner_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cleaner_node
)

ament_package()