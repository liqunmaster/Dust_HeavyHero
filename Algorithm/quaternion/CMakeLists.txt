target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_sources(app PRIVATE quaternion.cpp)

target_compile_options(app PRIVATE
    $<$<COMPILE_LANGUAGE:CXX>:-fno-threadsafe-statics>)
