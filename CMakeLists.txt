set(PROJECT_NAME "large-terrain-manager")

project(${PROJECT_NAME} CXX C)
cmake_minimum_required(VERSION 2.8)

# Include path for additional CMake library finding scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# We want the binaries to be easily accessible
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# Add the libraries

# OGRE
find_package(OGRE REQUIRED)
link_directories(${OGRE_LIBRARY_DIRS})
include_directories(${OGRE_INCLUDE_DIRS})
list(APPEND LIBS ${OGRE_LIBRARIES})

# SDL
find_package(SDL2 REQUIRED)
link_directories(${SDL2_LIBRARY_DIRS})
include_directories(${SDL2_INCLUDE_DIR})
list(APPEND LIBS ${SDL2_LIBRARY})

add_definitions(-DOGRE_PLUGIN_DIR_REL="${OGRE_PLUGIN_DIR_REL}")
add_definitions(-DOGRE_PLUGIN_DIR_DBG="${OGRE_PLUGIN_DIR_DBG}")

# The project's include directories
file(GLOB PROJECT_INCLUDES
	source/
)
include_directories(${PROJECT_INCLUDES})

include_directories(.)

add_subdirectory (extern/shiny)
list(APPEND LIBS ${SHINY_LIBRARIES})

add_subdirectory (terrain)
list(APPEND LIBS "large-terrain")

# Boost
find_package(Boost COMPONENTS system thread filesystem wave REQUIRED) # Specify the required components
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
list(APPEND LIBS ${Boost_LIBRARIES})

add_subdirectory (demo)
