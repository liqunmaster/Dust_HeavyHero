target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(fdcan_channel)
add_subdirectory(imu_channel)
add_subdirectory(remote_channel)
