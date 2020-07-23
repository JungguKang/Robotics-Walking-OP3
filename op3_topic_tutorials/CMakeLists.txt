cmake_minimum_required(VERSION 2.8.3)
project(op3_topic_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
  robotis_controller_msgs
  op3_action_module_msgs
  op3_walking_module_msgs
  op3_online_walking_module_msgs
  op3_tuning_module_msgs
  cmake_modules
  dynamixel_sdk
  robotis_framework_common
  robotis_device
  robotis_controller
  robotis_math
  op3_kinematics_dynamics
  op3_balance_control
  open_cr_module
  op3_base_module
  op3_head_control_module
  op3_walking_module
  op3_action_module
  op3_direct_control_module
  op3_online_walking_module
  op3_tuning_module
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(FILES PlotData.msg)
generate_messages(DEPENDENCIES std_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  std_msgs 
  roscpp 
  robotis_controller_msgs 
  op3_action_module_msgs 
  op3_walking_module_msgs 
  op3_online_walking_module_msgs 
  cmake_modules 
  dynamixel_sdk 
  robotis_framework_common 
  robotis_device 
  robotis_controller 
  robotis_math 
  op3_kinematics_dynamics 
  op3_balance_control 
  open_cr_module 
  op3_base_module 
  op3_head_control_module 
  op3_walking_module 
  op3_action_module 
  op3_direct_control_module 
  op3_online_walking_module 
  op3_tuning_module 
  op3_tuning_module_msgs
)


###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/topic_subscriber.cpp src/balanceFeedback.cpp src/walking_state.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############
#install(TARGETS ${PROJECT_NAME}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#install(DIRECTORY config launch
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#############
## Testing ##
#############

