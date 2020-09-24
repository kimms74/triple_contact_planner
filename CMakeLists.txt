cmake_minimum_required(VERSION 2.8.3)
project(triple_contact_planner)

add_compile_options(-std=c++11)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
message(STATUS "CMAKE_MODULE_PATH: " ${CMAKE_MODULE_PATH})

find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

# find_package(regrasp_constraint_planner REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES triple_contact_planner
  CATKIN_DEPENDS roscpp tf2 tf2_geometry_msgs
  #  DEPENDS system_lib
)

#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib

  src/contact/contact.cpp
  src/contact_model/contact_model.cpp

  src/solver/contact_optimization_solver.cpp
  src/solver/contact_optimization.cpp
  src/solver/qp_solver.cpp
  src/solver/constraint/constraint_base.cpp
  src/solver/constraint/constraint_equality.cpp
  src/solver/constraint/constraint_inequality.cpp

  # src/contact_model_graph.cpp
  # src/contact_model/box_contact_model.cpp
  # src/contact_model/visualization/contact_model_visualization.cpp

)

add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(triple_contact_planner regrasp_constraint_planner)

# link_directories("/home/jiyeong/catkin_ws/devel/lib")
#set_property(TARGET triple_contact_planner PROPERTY POSITION_INDEPENDENT_CODE ON)
target_link_libraries (${PROJECT_NAME}_lib
  qpOASES
)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries (${PROJECT_NAME}
  ${catkin_LIBRARIES}
  qpOASES
  ${PROJECT_NAME}_lib
  yaml-cpp
)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# add_executable(filtering src/filter_by_distance.cpp)
# target_link_libraries (filtering
#   ${catkin_LIBRARIES}
#   qpOASES
#   triple_contact_planner
#   yaml-cpp
  
# )
