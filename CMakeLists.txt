# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(hokuyo_node)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs driver_base self_test diagnostic_updater dynamic_reconfigure)

# Dynamic reconfigure support
generate_dynamic_reconfigure_options(cfg/Hokuyo.cfg)

include_directories(include cfg/cpp ${catkin_INCLUDE_DIRS})

# Hokuyo library
add_library(libhokuyo src/hokuyo.cpp)

# Hokuyo_node requires boost::thread
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

# hokuyo_node
add_executable(hokuyo_node src/hokuyo_node.cpp)
add_dependencies(hokuyo_node ${PROJECT_NAME}_gencfg)
target_link_libraries(hokuyo_node libhokuyo ${Boost_LIBRARIES} ${catkin_LIBRARIES})

# Utilities that use log4cxx - getID and getFirmwareVersion
find_library(LOG4CXX_LIBRARY log4cxx)
if(NOT LOG4CXX_LIBRARY)
  message(FATAL_ERROR "Couldn't find log4cxx library")
endif()

add_executable(getID src/getID.cpp)
target_link_libraries(getID libhokuyo ${LOG4CXX_LIBRARY} ${catkin_LIBRARIES})

add_executable(getFirmwareVersion src/getFirmwareVersion.cpp)
target_link_libraries(getFirmwareVersion libhokuyo ${LOG4CXX_LIBRARY} ${catkin_LIBRARIES})

# TODO: fill in what other packages will need to use this package
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    DEPENDS
    CATKIN-DEPENDS roscpp sensor_msgs driver_base self_test diagnostic_updater dynamic_reconfigure
    INCLUDE_DIRS include
    LIBRARIES libhokuyo ${LOG4CXX_LIBRARY} ${Boost_LIBRARIES}
)