cmake_minimum_required(VERSION 2.8.3)
project(ros_openalpr)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp
                                        std_msgs
                                        sensor_msgs
                                        image_transport
                                        message_generation)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
                  FILES
                  PlateCandidate.msg
                  PlateResult.msg
                  PlatesResults.msg)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES sensor_msgs std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project
catkin_package(CATKIN_DEPENDS
               roscpp
               std_msgs
               sensor_msgs
               image_transport
               message_runtime)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

## Build options
add_compile_options(-std=c++11)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Werror)
add_compile_options(-Wconversion)
add_compile_options(-Wshadow)
add_compile_options(-O3)
add_compile_options(-g)
add_compile_options(-flto)

## Node
add_executable(${PROJECT_NAME}_node src/ros_openalpr_node.cpp)
add_dependencies(${PROJECT_NAME}_node
                 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} openalpr)

#############
## Install ##
#############

## Mark library for installation
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
