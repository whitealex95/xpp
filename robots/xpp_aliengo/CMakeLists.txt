cmake_minimum_required(VERSION 2.8.3)
project(xpp_aliengo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  xpp_vis
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/aliengoleg_inverse_kinematics.cc
  src/inverse_kinematics_aliengo.cc
)

## URDF visualizers for Aliengo
add_executable(urdf_visualizer_aliengo src/exe/urdf_visualizer_aliengo.cc)
target_link_libraries(urdf_visualizer_aliengo
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(
  TARGETS ${PROJECT_NAME} urdf_visualizer_aliengo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation
install(
  DIRECTORY launch rviz meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
