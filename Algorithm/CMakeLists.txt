target_include_directories(app PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(buffer)

if(CONFIG_REMOTE_DEVICE_VT03)
    add_subdirectory(crc)
endif()
