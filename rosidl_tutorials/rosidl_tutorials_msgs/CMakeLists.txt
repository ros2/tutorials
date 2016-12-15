cmake_minimum_required(VERSION 3.5)

project(rosidl_tutorials_msgs)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)
# find the ROS message code generators
 find_package(rosidl_default_generators REQUIRED)

# declare the message files to generate code for
 set(msg_files
   "msg/Contact.msg"
   "msg/NewInROS2.msg"
 )

# declare the service files to generate code for
 set(srv_files
   "srv/AddTwoFloats.srv"
 )

 rosidl_generate_interfaces(${PROJECT_NAME}
   ${msg_files}
   ${srv_files}
 )

 ament_export_dependencies(rosidl_default_runtime)

ament_package()
