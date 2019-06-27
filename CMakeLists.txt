cmake_minimum_required(VERSION 2.8.3)

project(ds_based_contact_tasks)

# Add support for c++11
add_definitions(-std=c++11 -O3)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  gazebo_ros
  std_msgs
  message_generation
  kdl_parser
  dynamic_reconfigure
  svm_grad
  sg_differentiation
  utils
  lwr_controllers
)

find_package(Eigen 3 QUIET) 
if(NOT Eigen_FOUND)
  include(cmake/FindEigen3.cmake)
endif() 

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

# Generate dynamic reconfigure parameters
generate_dynamic_reconfigure_options(cfg/surfacePolishing_params.cfg
                                     cfg/objectGrasping_params.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs message_generation kdl_parser dynamic_reconfigure svm_grad sg_differentiation utils lwr_controllers
  # DEPENDS Eigen
 )

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(surface_learning src/surfaceLearning_node.cpp src/SurfaceLearning.cpp)
target_link_libraries(surface_learning ${catkin_LIBRARIES})
add_dependencies(surface_learning ${PROJECT_NAME}_gencpp)

add_executable(surface_polishing src/surfacePolishing_node.cpp src/SurfacePolishing.cpp src/adaptation_rbf.cpp src/RbfAdaptation.cpp)
target_link_libraries(surface_polishing ${catkin_LIBRARIES})
add_dependencies(surface_polishing ${PROJECT_NAME}_gencpp)

add_executable(two_robots_transform src/twoRobotsTransform_node.cpp src/TwoRobotsTransform.cpp)
target_link_libraries(two_robots_transform ${catkin_LIBRARIES})
add_dependencies(two_robots_transform ${PROJECT_NAME}_gencpp)

add_executable(object_grasping src/objectGrasping_node.cpp src/ObjectGrasping.cpp src/Workspace.cpp)
target_link_libraries(object_grasping ${catkin_LIBRARIES})
add_dependencies(object_grasping ${PROJECT_NAME}_gencpp)

FILE(GLOB SRCS src/SurfaceLearning.cpp src/SurfacePolishing.cpp src/TwoRobotsTransform.cpp src/ObjectGrasping.cpp)
FILE(GLOB INCS include/SurfaceLearning.h include/SurfacePolishing.h include/TwoRobotsTransform.h include/ObjectGrasping.h)

add_library(${PROJECT_NAME} ${SRCS} ${INCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})