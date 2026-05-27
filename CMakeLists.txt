cmake_minimum_required(VERSION 3.8)
project(line_follower_robot)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  line_follower_robot/line_follower_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
