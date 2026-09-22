target_sources(app PRIVATE bsp_gpio.c)
target_include_directories(app PUBLIC ${CMAKE_CURRENT_LIST_DIR})
