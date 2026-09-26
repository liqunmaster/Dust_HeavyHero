target_sources(app PRIVATE dji_motor.cpp)
target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR})

add_subdirectory(c610)
add_subdirectory(c620)
