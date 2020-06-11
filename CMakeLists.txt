project(ExtendedKF)
cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

# ###########################
# CODE                      #
# ###########################
set(sources  src/tools.cpp src/FusionEKF.cpp src/kalman_filter.cpp src/tools.h src/FusionEKF.h src/kalman_filter.h)

# add EKF library
add_library(${PROJECT_NAME}_library ${sources})
target_include_directories(${PROJECT_NAME}_library PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME}_library z ssl uv uWS)

# main
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${PROJECT_NAME}_library)


# ###########################
# Testing                   #
# ###########################
enable_testing()
find_package(GTest REQUIRED)
include_directories(src/ ${gtest_INCLUDE_DIRS})

# all test/*.cpp files
file(GLOB_RECURSE TEST_SRC_FILES RELATIVE ${PROJECT_SOURCE_DIR} test/*.cpp)

# tests
add_executable(${PROJECT_NAME}_test ${TEST_SRC_FILES})
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_library ${GTEST_BOTH_LIBRARIES} pthread)
add_test(${PROJECT_NAME}_all_tests ${PROJECT_NAME}_test)
