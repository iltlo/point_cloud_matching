cmake_minimum_required(VERSION 3.8)
project(point_cloud_matching)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)


# Generate service interfaces (this handles the Python bindings for .srv)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MatchClouds.srv"
  DEPENDENCIES geometry_msgs
)

# Install the project modules alongside the generated service bindings.
install(FILES
  point_cloud_matching/__init__.py
  point_cloud_matching/matching_engine.py
  point_cloud_matching/preprocessing.py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)

# Install the node as an executable
install(PROGRAMS
  point_cloud_matching/matching_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME matching_node
)
install(PROGRAMS
  point_cloud_matching/match_visualizer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME visualizer_node
)

# Install launch and config
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()