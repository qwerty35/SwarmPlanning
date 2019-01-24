cmake_minimum_required(VERSION 2.8.3)
project(swarm_planner)

set(CMAKE_CXX_STANDARD 14)

include_directories(${PROJECT_SOURCE_DIR}/include)
#set(CMAKE_BUILD_TYPE Release)

# THIRD PARTY
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/ecbs)
include_directories(${PROJECT_SOURCE_DIR}/third_party/ecbs/include)

#BOOST
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)

#EIGEN
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})

#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

#OCTOMAP
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})
add_definitions(-DOCTOMAP_NODEBUGOUT)

#EDT
find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
link_libraries(${DYNAMICEDT3D_LIBRARIES})

#CPLEX
add_definitions(-DNDEBUG)
add_definitions(-DIL_STD)
set(CPLEX_PREFIX_DIR      /opt/ibm/ILOG/CPLEX_Studio128)
set(CPLEX_INCLUDE_DIR     ${CPLEX_PREFIX_DIR}/cplex/include)
set(CPLEX_LIBRARIES_DIR   ${CPLEX_PREFIX_DIR}/cplex/lib/x86-64_linux/static_pic)
set(CONCERT_INCLUDE_DIR   ${CPLEX_PREFIX_DIR}/concert/include)
set(CONCERT_LIBRARIES_DIR ${CPLEX_PREFIX_DIR}/concert/lib/x86-64_linux/static_pic)
set(ILOCPLEX_INCLUDE_DIR  ${CPLEX_PREFIX_DIR}/)
include_directories(${CPLEX_INCLUDE_DIR} ${CONCERT_INCLUDE_DIR})
link_directories(${CPLEX_LIBRARIES_DIR} ${CONCERT_LIBRARIES_DIR})

#CATKIN
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  octomap_ros
  octomap_msgs
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

# BUILD
add_executable(init_traj_generator src/init_traj_generator.cpp)
target_link_libraries(init_traj_generator
  ${catkin_LIBRARIES}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(random_map_generator src/random_map_generator.cpp)
target_link_libraries(random_map_generator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(blend src/blend.cpp)
target_link_libraries(blend
    m
    ilocplex
    cplex
    concert
    pthread
    dl
)
