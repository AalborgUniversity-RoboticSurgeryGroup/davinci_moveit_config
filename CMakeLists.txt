cmake_minimum_required(VERSION 2.8.3)
project(davinci_moveit_config)

find_package(catkin REQUIRED)

catkin_package()
########## TEST START! ############
## Find catkin and any catkin packages - and others packages
find_package(catkin REQUIRED COMPONENTS roscpp moveit_ros_planning_interface)
find_package(cmake_modules REQUIRED)
## Find the Eigen package
find_package(Eigen REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp moveit_ros_planning_interface rospy std_msgs message_generation)

include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
########## TEST END! ##############


### start from tutorial:
#add_message_files(DIRECTORY msg FILES Num.msg)
#add_service_files(DIRECTORY srv FILES AddTwoInts.srv)
#generate_messages(DEPENDENCIES std_msgs)
### end from tutorial

catkin_package(CATKIN_DEPENDS message_runtime)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

########## TEST START! ##############
## Build the executable (located by default at ~/catkin_ws/devel/lib/share/<package name> - in our case at ~/groovy_workspace/build/kuka_lbr_simple_moveit_generated/CMakeFiles/MoveGroupInterface.dir/src)
add_executable(MoveGroupInterfaceExecute src/MoveGroupInterface_new.cpp)
target_link_libraries(MoveGroupInterfaceExecute ${catkin_LIBRARIES})
#add_dependencies(MoveGroupInterface_new davinci_moveit_config_generate_messages_cpp)

#add_executable(listener davinci_moveit_config_generate_messages_cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener davinci_moveit_config_generate_messages_cpp)
######### TEST END #################
