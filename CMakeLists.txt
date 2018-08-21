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
set(xsens_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/lib/include ${PROJECT_SOURCE_DIR}/include)
set(xsens_LIBRARY ${PROJECT_SOURCE_DIR}/lib/xstypes/libxstypes.a)
set(xsens_comm_LIBRARY ${PROJECT_SOURCE_DIR}/lib/xcommunication/libxcommunication.a)
set(xsens_bridge_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include)
find_library(xsens_api_LIBRARY xsensdeviceapi PATH ${xsens_LINK_DIRS})
find_library(xsens_types_LIBRARY xstypes PATH ${xsens_LINK_DIRS})

#########
# BUILD #
#########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${xsens_INCLUDE_DIRS}
)

add_custom_target(
   xsens_INCLUDE_LIB
   COMMAND make
   WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/lib/
)

add_executable(xsens_bridge
               src/xsens_bridge.cpp
               src/deviceclass.cpp)
#Should probably make a list for these and just loop over itd
set(EXEC_LIST xsens_bridge)
foreach(EXEC_FILE ${EXEC_LIST})
  target_link_libraries(${EXEC_FILE} ${catkin_LIBRARIES} ${xsens_api_LIBRARY} ${xsens_types_LIBRARY} ${xsens_LIBRARY} ${xsens_comm_LIBRARY})
  add_dependencies(${EXEC_FILE} ${catkin_EXPORTED_TARGETS} ${xsens_INCLUDE_LIB})
  install(TARGETS ${EXEC_FILE}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endforeach(EXEC_FILE ${EXEC_LIST})





                             
