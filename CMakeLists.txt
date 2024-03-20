cmake_minimum_required(VERSION 3.5.1)
cmake_policy(SET CMP0048 NEW)
project(custom_planner)

# add_compile_options(-std=c++11)

##############################################################################
# Find dependencies
##############################################################################

set(THIS_PACKAGE_ROS_DEPS roscpp costmap_2d nav_core pluginlib tf tf2
  geometry_msgs nav_msgs)
find_package(catkin REQUIRED COMPONENTS
  ${THIS_PACKAGE_ROS_DEPS} 
  geometry_msgs
  message_generation
  vda5050_msgs
  std_msgs
  nav_msgs
  roscpp
)

# Find SBPL
# find_package(SBPL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
# include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

# include_directories(include ${catkin_INCLUDE_DIRS} ${SBPL_INCLUDE_DIRS})

##############################################################################
# Generate messages
##############################################################################

add_message_files(FILES 
                  SBPLLatticePlannerStats_.msg 
                  OccupancyGrid_.msg)
add_service_files(
  DIRECTORY srv
  FILES
  PlanWithOrder.srv
)
generate_messages(DEPENDENCIES geometry_msgs nav_msgs vda5050_msgs)

##############################################################################
# Define package
##############################################################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS} message_runtime vda5050_msgs nav_msgs std_msgs geometry_msgs roscpp
  # DEPENDS SBPL
)

##############################################################################
# Build
##############################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SBPL_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME} 
  src/custom_planner.cpp
  src/pathway.cc
  src/pose.cc
  src/Curve_common.cpp
)
target_link_libraries(${PROJECT_NAME} ${SBPL_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_compile_options(${PROJECT_NAME} PUBLIC "-Wno-terminate")  # suppress warning from included SBPL header

##############################################################################
# Install
##############################################################################

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  launch
  matlab
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
MESSAGE(STATUS "This is include dir: " ${catkin_INCLUDE_DIRS})
MESSAGE(STATUS "This is catkin library: " ${catkin_LIBRARIES})