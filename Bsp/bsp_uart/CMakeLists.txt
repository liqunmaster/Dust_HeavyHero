target_sources(app PRIVATE
    bsp_uart.c
    uart.c
)
target_include_directories(app PUBLIC ${CMAKE_CURRENT_LIST_DIR})
