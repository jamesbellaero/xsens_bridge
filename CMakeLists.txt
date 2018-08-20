cmake_minimum_required(VERSION 3.0)
project(xsens_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  roscpp
)

add_message_files(DIRECTORY msg FILES
  Imu.msg
)

generate_messages( DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  std_msgs
  roscpp
  message_runtime
)

set(xsens_LINK_DIRS /usr/local/xsens/lib32)
set(xsens_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/lib/xcommunication ${PROJECT_SOURCE_DIR}/lib/xstypes)
find_library(xsens_LIBRARY xsensdeviceapi PATH ${xsens_LINK_DIRS})
find_library(xsens_types_LIBRARY xstypes PATH ${xsens_LINK_DIRS})

#########
# BUILD #
#########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${xsens_INCLUDE_DIRS}
)
add_custom_target(
   xcommunication_LIBRARY
   COMMAND make
   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/
)

add_executable(xsens_node
               src/xsens_node.cpp
               src/deviceclass.cpp)
#Should probably make a list for these and just loop over itd
set(EXEC_LIST xsens_node)
foreach(EXEC_FILE ${EXEC_LIST})
  target_link_libraries(${EXEC_FILE} ${catkin_LIBRARIES} ${xsens_LIBRARY} ${xsens_types_LIBRARY})
  add_dependencies(${EXEC_FILE} ${catkin_EXPORTED_TARGETS})
  install(TARGETS ${EXEC_FILE}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(EXEC_FILE ${EXEC_LIST})





                             
