cmake_minimum_required(VERSION 3.0.2)
project(commissioning_robot)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  tf
  tf2
  tf2_ros
  dynamic_reconfigure
  message_generation
)

# https://github.com/jsk-ros-pkg/jsk_visualization/issues/832#issuecomment-935734810


#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/robot_param.cfg
  cfg/motor_port.cfg
  cfg/encoder_port.cfg
  cfg/switch_port.cfg
  #...
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  FILES
  ControlState.msg
  FeedbackState.msg
  MechanismReport.msg
  PhaseReport.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

# includeしたいファイルがあるときはここにディレクトリを追加していく!
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(drive_inverse_kinematics src/drive_inverse_kinematics.cpp)
add_executable(rviz_publisher src/rviz_publisher.cpp)
add_executable(gazebo_publisher src/gazebo_publisher.cpp)
add_executable(sim_bridge src/sim_bridge.cpp)
add_executable(can_bridge src/can_bridge.cpp)
add_executable(arm_controller src/arm_controller.cpp)
add_executable(cable_manager_controller src/cable_manager_controller.cpp)
add_executable(drive_lift_controller src/drive_lift_controller.cpp)
add_executable(drive_wheel_controller src/drive_wheel_controller.cpp)
add_executable(main_lift_controller src/main_lift_controller.cpp)
add_executable(pose_controller src/pose_controller.cpp)
add_executable(robot_core src/robot_core.cpp)
add_executable(robot_tf_publisher src/robot_tf_publisher.cpp)


add_dependencies(can_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(arm_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(cable_manager_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(drive_lift_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(drive_wheel_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(main_lift_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(pose_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(robot_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sim_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(robot_tf_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# test sita no ya tu
add_dependencies(gazebo_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(drive_inverse_kinematics
  ${catkin_LIBRARIES}
)
target_link_libraries(rviz_publisher
  ${catkin_LIBRARIES}
)
target_link_libraries(gazebo_publisher
  ${catkin_LIBRARIES}
)
target_link_libraries(can_bridge
  ${catkin_LIBRARIES}
)
target_link_libraries(sim_bridge
  ${catkin_LIBRARIES}
)
target_link_libraries(arm_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(cable_manager_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(drive_lift_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(drive_wheel_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(main_lift_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(pose_controller
  ${catkin_LIBRARIES}
)
target_link_libraries(robot_core
  ${catkin_LIBRARIES}
)
target_link_libraries(robot_tf_publisher
  ${catkin_LIBRARIES}
)


# make sure configure headers are built before any node using them
add_dependencies(gazebo_publisher ${PROJECT_NAME}_gencfg)


add_dependencies(can_bridge ${PROJECT_NAME}_gencfg)
add_dependencies(arm_controller ${PROJECT_NAME}_gencfg)
add_dependencies(cable_manager_controller ${PROJECT_NAME}_gencfg)
add_dependencies(drive_lift_controller ${PROJECT_NAME}_gencfg)
add_dependencies(drive_wheel_controller ${PROJECT_NAME}_gencfg)
add_dependencies(main_lift_controller ${PROJECT_NAME}_gencfg)
add_dependencies(pose_controller ${PROJECT_NAME}_gencfg)
add_dependencies(robot_core ${PROJECT_NAME}_gencfg)
add_dependencies(sim_bridge ${PROJECT_NAME}_gencfg)


catkin_install_python(PROGRAMS scripts/rviz_jsk.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/hana_camera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ROS Custom message generation 
# https://qiita.com/srs/items/7ac023c549e585caeed0