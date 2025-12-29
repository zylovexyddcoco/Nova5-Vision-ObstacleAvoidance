cmake_minimum_required(VERSION 3.5)
project(obstacle_avoidance)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/octomap_to_planning_scene.py
  scripts/self_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
