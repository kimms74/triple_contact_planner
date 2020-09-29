cmake_minimum_required(VERSION 2.6)

project(triple_contact_planner)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# add_compile_options(-std=c++11)

LIST( APPEND CMAKE_MODULE_PATH "$ENV{HOME}/cmake" )
add_definitions(-DBOOST_MATH_NO_LONG_DOUBLE_MATH_FUNCTIONS)
# list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  eigenpy
  roslib
)

find_package(Boost REQUIRED COMPONENTS system python)
find_package(PythonLibs 2.7 REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

catkin_python_setup()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES triple_contact_planner
  CATKIN_DEPENDS 
    roscpp 
    tf2 
    tf2_geometry_msgs
  #  DEPENDS system_lib
)

#set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  /usr/include/boost
)
include_directories(${PYTHON_INCLUDE_DIRS})

link_directories(${Boost_LIBRARY_DIR})

add_library(${PROJECT_NAME}_lib
  src/contact/contact.cpp
  src/contact_model/contact_model.cpp
  src/solver/contact_optimization_solver.cpp
  src/solver/contact_optimization.cpp
  src/solver/qp_solver.cpp
  src/solver/constraint/constraint_base.cpp
  src/solver/constraint/constraint_equality.cpp
  src/solver/constraint/constraint_inequality.cpp
)

add_library(python_wrapper_cpp 
  src/wrapper/python_wrapping.cpp
)

add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries (${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}  
  qpOASES
)

target_link_libraries(python_wrapper_cpp ${PROJECT_NAME}_lib
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}  
  yaml-cpp
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

set_target_properties(python_wrapper_cpp PROPERTIES
PREFIX ""
LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)