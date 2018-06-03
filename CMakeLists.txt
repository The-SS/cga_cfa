cmake_minimum_required(VERSION 2.8.3)
project(cga_cfa)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  )

catkin_package()

add_service_files(
  FILES
  CCDA.srv
)
