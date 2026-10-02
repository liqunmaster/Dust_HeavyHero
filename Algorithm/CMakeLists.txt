target_include_directories(app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(buffer)
add_subdirectory(crc)
add_subdirectory(kalman)
add_subdirectory(quaternion)
