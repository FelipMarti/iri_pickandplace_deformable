cmake_minimum_required(VERSION 2.8.3)
project(iri_pickandplace_deformable)

## Find catkin macros and libraries
find_package(catkin REQUIRED)
# ******************************************************************** 
#                 Add catkin additional components here
# ******************************************************************** 
find_package(catkin REQUIRED COMPONENTS iri_base_algorithm iri_action_server iri_wam_generic_pickandplace actionlib iri_common_drivers_msgs iri_action_server std_msgs message_generation actionlib_msgs tf)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# ******************************************************************** 
#           Add system and labrobotica dependencies here
# ******************************************************************** 
# find_package(<dependency> REQUIRED)

# ******************************************************************** 
#           Add topic, service and action definition here
# ******************************************************************** 
## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   PaPDeformable.action 
#   Action2.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs actionlib_msgs # Or other packages containing msgs
 )

# ******************************************************************** 
#                 Add the dynamic reconfigure file 
# ******************************************************************** 
generate_dynamic_reconfigure_options(cfg/PickandplaceDeformable.cfg)

# ******************************************************************** 
#                 Add run time dependencies here
# ******************************************************************** 
catkin_package(
#  INCLUDE_DIRS 
#  LIBRARIES 
# ******************************************************************** 
#            Add ROS and IRI ROS run time dependencies
# ******************************************************************** 
 CATKIN_DEPENDS iri_base_algorithm iri_action_server iri_action_server iri_wam_generic_pickandplace actionlib iri_common_drivers_msgs std_msgs message_generation message_runtime actionlib_msgs tf
# ******************************************************************** 
#      Add system and labrobotica run time dependencies here
# ******************************************************************** 
#  DEPENDS 
)

###########
## Build ##
###########
# This hack is only to allow the rosmsg tool to locate the messages genererated by the actions, so that the IRI scripts can
# locate them when adding actions to a node.
add_custom_target(iri_pickandplace_deformable_copy_action_msgs ALL)
add_custom_command(TARGET iri_pickandplace_deformable_copy_action_msgs POST_BUILD
                   COMMAND rsync -aq ${CATKIN_DEVEL_PREFIX}/share/${PROJECT_NAME}/msg/ ${CMAKE_CURRENT_SOURCE_DIR}/msg/)
add_dependencies(iri_pickandplace_deformable_copy_action_msgs iri_pickandplace_deformable_generate_messages_cpp)
# ******************************************************************** 
#                   Add the include directories 
# ******************************************************************** 
include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})
# include_directories(${<dependency>_INCLUDE_DIR})

## Declare a cpp library
# add_library(${PROJECT_NAME} <list of source files>)

## Declare a cpp executable
add_executable(${PROJECT_NAME} src/pickandplace_deformable_alg.cpp src/pickandplace_deformable_alg_node.cpp)

# ******************************************************************** 
#                   Add the libraries
# ******************************************************************** 
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
# target_link_libraries(${PROJECT_NAME} ${<dependency>_LIBRARY})

# ******************************************************************** 
#               Add message headers dependencies 
# ******************************************************************** 
# add_dependencies(${PROJECT_NAME} <msg_package_name>_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} iri_wam_generic_pickandplace_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} iri_common_drivers_msgs_generate_messages_cpp)
# ******************************************************************** 
#               Add dynamic reconfigure dependencies 
# ******************************************************************** 
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
