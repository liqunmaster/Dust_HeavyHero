target_sources(app PRIVATE
    double_buffer.cpp
    ring_buffer.cpp
)
target_include_directories(app PUBLIC ${CMAKE_CURRENT_LIST_DIR})
