cmake_minimum_required(VERSION 2.4.6)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")
set(CMAKE_BUILD_TYPE Debug)
add_executable(vision_testing src/vision_testing.cpp)
target_link_libraries(vision_testing ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})