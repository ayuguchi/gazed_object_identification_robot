cmake_minimum_required(VERSION 2.8.3)
project(person_tracking_kalman)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf
  message_filters
  sensor_msgs
	cmake_modules
)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES person_tracking_kalman
  CATKIN_DEPENDS roscpp
	#DEPENDS Eigen
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
set(EIGEN_INC_DIR /usr/include/eigen3/)
include_directories(
	include
	#${EIGEN_INC_DIR}
	${EIGEN_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(person_tracking_kalman
   src/kalman.cpp
 )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(person_tracking_filter_kalman ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(person_tracking_kalman_node src/person_tracking_kalman_node.cpp src/kalman.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(person_tracking_filter_kalman_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(person_tracking_kalman_node
   ${catkin_LIBRARIES}
	 ${EIGEN3_LIBS}
 )


