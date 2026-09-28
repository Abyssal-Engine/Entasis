cmake_minimum_required(VERSION 3.20)
project(entasis_sdk_consumer LANGUAGES C CXX)
find_package(Entasis CONFIG REQUIRED)
enable_testing()

add_executable(entasis_c_shared minimal.c)
target_compile_features(entasis_c_shared PRIVATE c_std_11)
target_link_libraries(entasis_c_shared PRIVATE Entasis::cooking)

add_executable(entasis_cpp_shared minimal.cpp)
target_compile_features(entasis_cpp_shared PRIVATE cxx_std_20)
target_link_libraries(entasis_cpp_shared PRIVATE Entasis::cooking)

add_executable(entasis_c_static minimal.c)
target_compile_features(entasis_c_static PRIVATE c_std_11)
target_link_libraries(entasis_c_static PRIVATE Entasis::cooking_static)

add_executable(entasis_cpp_static minimal.cpp)
target_compile_features(entasis_cpp_static PRIVATE cxx_std_20)
target_link_libraries(entasis_cpp_static PRIVATE Entasis::cooking_static)

foreach(consumer entasis_c_shared entasis_cpp_shared entasis_c_static entasis_cpp_static)
    add_test(NAME ${consumer} COMMAND ${consumer})
endforeach()

if(WIN32)
    foreach(consumer entasis_c_shared entasis_cpp_shared)
        add_custom_command(TARGET ${consumer} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:Entasis::runtime> $<TARGET_FILE:Entasis::cooking>
                $<TARGET_FILE_DIR:${consumer}>)
    endforeach()
endif()
