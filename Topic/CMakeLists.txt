target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(fdcan_chan)
add_subdirectory(imu_chan)
add_subdirectory(remote_chan)
