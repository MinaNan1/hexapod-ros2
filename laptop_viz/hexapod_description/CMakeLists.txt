cmake_minimum_required(VERSION 3.8)
project(hexapod_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install only the dirs present in this viz-only package (urdf + meshes are
# what RViz needs to resolve package:// mesh paths; rviz holds the layout).
# The full Jetson package also has launch/ — intentionally omitted here.
install(DIRECTORY
  urdf
  meshes
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()