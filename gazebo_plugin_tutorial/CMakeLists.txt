cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(hello_world SHARED hello_world.cc)
add_library(model_push SHARED model_push.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(model_push ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
