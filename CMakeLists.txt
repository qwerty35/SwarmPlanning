cmake_minimum_required(VERSION 2.8.3)
project(swarm_planner)

set(CMAKE_CXX_STANDARD 14)

include_directories(${PROJECT_SOURCE_DIR}/include)
set(CMAKE_BUILD_TYPE Release)

# THIRD PARTY
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/ecbs)
include_directories(${PROJECT_SOURCE_DIR}/third_party/ecbs/include)

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

#BOOST
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)

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
  ${PCL_LIBRARIES}
  ${OCTOMAP_INCLUDE_DIRS}
)
add_executable(sfc_generator src/sfc_generator.cpp)
target_link_libraries(sfc_generator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(obstacle_box_generator src/obstacle_box_generator.cpp)
target_link_libraries(obstacle_box_generator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(random_map_generator src/random_map_generator.cpp)
target_link_libraries(random_map_generator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(init_cond_publisher src/init_cond_publisher.cpp)
target_link_libraries(init_cond_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
