cmake_minimum_required(VERSION 2.8.3)
project(stabil)


find_package (Python3 COMPONENTS Interpreter Development)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Num.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES AddTwoInts.srv ServoServer.srv PCA9685.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stabil
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
	CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker stabil_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener stabil_generate_messages_cpp)

add_executable(servo_client src/servo_client.cpp)
target_link_libraries(servo_client ${catkin_LIBRARIES})
add_dependencies(servo_client stabil_gencpp)

add_executable(network_server src/network_server.cpp)
target_link_libraries(network_server ${catkin_LIBRARIES})
add_dependencies(network_server stabil_gencpp)

add_executable(control_listener src/control_listener.cpp src/MyServo.cpp)
target_link_libraries(control_listener ${catkin_LIBRARIES})
add_dependencies(control_listener stabil_gencpp)

add_executable(teleop_bot src/teleop_bot.cpp)
target_link_libraries(teleop_bot ${catkin_LIBRARIES})
add_dependencies(teleop_bot stabil_gencpp)

add_executable(pca9685_server src/pca9685_server.cpp include/JHPWMPCA9685.cpp)
target_link_libraries(pca9685_server ${catkin_LIBRARIES})
add_dependencies(pca9685_server stabil_gencpp)



catkin_install_python(
	PROGRAMS scripts/talker.py scripts/listener.py scripts/servo_client.py scripts/add_two_ints_client.py scripts/add_two_ints_server.py
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
