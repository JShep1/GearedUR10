# setup some things initially
cmake_minimum_required (VERSION 2.6)
project (standing)
include (CheckIncludeFiles)
include (CheckLibraryExists)
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/CMakeModules)
set (CMAKE_BUILD_TYPE Debug)

# find libraries
find_package (Ravelin REQUIRED)
find_package (Moby REQUIRED)
find_package (Boost REQUIRED)

# setup include directories, compiler flags, and libraries for required pkgs
include_directories (${BOOST_INCLUDE_DIR})
include_directories (${RAVELIN_INCLUDE_DIR})
include_directories (${MOBY_INCLUDE_DIR})

# plugins
# sphere plugin
add_library(resting-controller MODULE resting-controller.cpp)
add_library(resting-controller-PID MODULE resting-controller-PID.cpp)
add_library(sinusoidal-controller MODULE sinusoidal-controller.cpp)
target_link_libraries(resting-controller ${MOBY_LIBRARIES} ${RAVELIN_LIBRARIES})
target_link_libraries(resting-controller-PID ${MOBY_LIBRARIES} ${RAVELIN_LIBRARIES})
target_link_libraries(sinusoidal-controller ${MOBY_LIBRARIES} ${RAVELIN_LIBRARIES})


