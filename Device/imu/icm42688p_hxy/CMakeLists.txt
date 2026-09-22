target_sources(app PRIVATE icm42688p_hxy.cpp)
target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
)
