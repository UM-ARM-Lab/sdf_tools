cmake_minimum_required(VERSION 2.8.3)
project(sdf_tools)

option(SDF_TOOLS_PYTHON_BINDINGS ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(REQ_CATKIN_PACKAGES
    rospy
    std_msgs
    sensor_msgs
    visualization_msgs
    image_transport
    cv_bridge
    arc_utilities)

find_package(catkin REQUIRED COMPONENTS ${REQ_CATKIN_PACKAGES} message_generation OPTIONAL_COMPONENTS pybind11_catkin)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
find_package(OpenCV REQUIRED)
# MoveIt! isn't a required dependency
find_package(moveit_msgs)
find_package(moveit_core)
find_package(moveit_ros_planning)

catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg FILES
    SDF.msg
    CollisionMap.msg
    TaggedObjectCollisionMap.msg)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv FILES
    ComputeSDF.srv)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
    geometry_msgs
    std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
if(moveit_core_FOUND)
    catkin_package(
        INCLUDE_DIRS    include
        LIBRARIES       ${PROJECT_NAME}
        CATKIN_DEPENDS  ${REQ_CATKIN_PACKAGES}
                        message_runtime
                        moveit_core
                        moveit_ros_planning
                        moveit_msgs
        DEPENDS         Eigen3
                        OpenCV)
else()
    catkin_package(
        INCLUDE_DIRS    include
        LIBRARIES       ${PROJECT_NAME}
        CATKIN_DEPENDS  ${REQ_CATKIN_PACKAGES}
                        message_runtime
        DEPENDS         Eigen3
                        OpenCV)
endif()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
if(moveit_core_FOUND)
    include_directories(include SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${moveit_core_INCLUDE_DIRS}
        ${moveit_msgs_INCLUDE_DIRS}
        ${moveit_ros_planning_INCLUDE_DIRS})
else()
    include_directories(include SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS})
endif()

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -flto -O3 -Wall -Wextra -Werror -Wshadow")

if(moveit_core_FOUND)
    message(STATUS "MoveIt! found. Building SDF Tools library with MoveIt! integration.")
    # SDF library
    add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}/collision_map.cpp
        src/${PROJECT_NAME}/tagged_object_collision_map.cpp
        src/${PROJECT_NAME}/dynamic_spatial_hashed_collision_map.cpp
        src/${PROJECT_NAME}/sdf.cpp
        src/${PROJECT_NAME}/sdf_builder.cpp)
    add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencpp)
    target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${moveit_core_LIBRARIES}
        ${moveit_msgs_LIBRARIES}
        ${moveit_ros_planning_LIBRARIES})
    # SDF generation node
    add_executable(sdf_generation_node src/sdf_generation_node.cpp)
    add_dependencies(sdf_generation_node
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencpp)
    target_link_libraries(sdf_generation_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
        ${moveit_core_LIBRARIES}
        ${moveit_msgs_LIBRARIES}
        ${moveit_ros_planning_LIBRARIES})
    # Headers
    add_custom_target(${PACKAGE_NAME}_headers SOURCES
        include/${PROJECT_NAME}/collision_map.hpp
        include/${PROJECT_NAME}/tagged_object_collision_map.hpp
        include/${PROJECT_NAME}/dynamic_spatial_hashed_collision_map.hpp
        include/${PROJECT_NAME}/sdf.hpp
        include/${PROJECT_NAME}/sdf_generation.hpp
        include/${PROJECT_NAME}/topology_computation.hpp
        include/${PROJECT_NAME}/sdf_builder.hpp)
else()
    message(STATUS "MoveIt! not found. Building SDF Tools library without MoveIt! integration.")
    # SDF library
    add_library(${PROJECT_NAME}
        src/${PROJECT_NAME}/collision_map.cpp
        src/${PROJECT_NAME}/tagged_object_collision_map.cpp
        src/${PROJECT_NAME}/dynamic_spatial_hashed_collision_map.cpp
        src/${PROJECT_NAME}/sdf.cpp)
    add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencpp)
    target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES})
    # Headers
    add_custom_target(${PACKAGE_NAME}_headers SOURCES
        include/${PROJECT_NAME}/collision_map.hpp
        include/${PROJECT_NAME}/tagged_object_collision_map.hpp
        include/${PROJECT_NAME}/dynamic_spatial_hashed_collision_map.hpp
        include/${PROJECT_NAME}/sdf.hpp)
endif()

# Simple test nodes for voxel_grid
add_executable(voxel_grid_test src/test_voxel_grid.cpp)
add_dependencies(voxel_grid_test ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(voxel_grid_test ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(estimate_distance_test src/estimate_distance_test.cpp)
add_dependencies(estimate_distance_test ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(estimate_distance_test ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(compute_convex_segments_test src/compute_convex_segments_test.cpp)
add_dependencies(compute_convex_segments_test ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(compute_convex_segments_test ${catkin_LIBRARIES} ${PROJECT_NAME})

# Image to SDF node
add_executable(image_2d_sdf_node src/image_2d_sdf_node.cpp)
add_dependencies(image_2d_sdf_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_2d_sdf_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Tutorial on how to use the components of the sdf_tools library
add_executable(sdf_tools_tutorial src/sdf_tools_tutorial.cpp)
add_dependencies(sdf_tools_tutorial ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(sdf_tools_tutorial ${catkin_LIBRARIES} ${PROJECT_NAME})


#####################
## Python Bindings ##
#####################

if (${SDF_TOOLS_PYTHON_BINDINGS})
    pybind11_add_module(pysdf_tools src/sdf_tools/bindings.cpp)
    target_link_libraries(pysdf_tools sdf_tools)
    # Suppress prefix "lib" because Python does not allow this prefix
    set_target_properties(pysdf_tools PROPERTIES
            LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
endif()

#############
## Install ##
#############

## Mark library for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
