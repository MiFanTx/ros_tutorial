cmake_minimum_required(VERSION 3.8)
project(ur3e_basics)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickPlace.action"
  DEPENDENCIES geometry_msgs builtin_interfaces action_msgs
)


# Install Python executables
install(PROGRAMS
  ur3e_basics/joint_listener.py
  ur3e_basics/joint_commander.py
  ur3e_basics/pick_place_server.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
