cmake_minimum_required(VERSION 2.8.3)
project(bms_status)
set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(
        catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation
)
find_package(serial REQUIRED)
#find_library(SERIAL serial)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES sensor_batterystate
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

# include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} ${serial_INCLUDE_DIRS} )
include_directories(${PROJECT_SOURCE_DIR}/include ${catkin_INCLUDE_DIRS} )

# add_executable(bms_rtu_node src/ros_modbus_rtu.cpp
#                             src/main.cpp)
add_executable(bms_rtu_node src/bms_modbus.cpp
                            src/main.cpp)
target_link_libraries(bms_rtu_node ${catkin_LIBRARIES} ${serial_LIBRARIES} modbus)


install(TARGETS bms_rtu_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )
