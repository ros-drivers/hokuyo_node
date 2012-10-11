rosbuild_add_boost_directories() 

rosbuild_add_executable(hokuyo_node hokuyo_node.cpp)
target_link_libraries(hokuyo_node libhokuyo)
rosbuild_link_boost(hokuyo_node thread)
