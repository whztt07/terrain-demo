set(PROJECT_NAME "large-terrain-manager")

project(${PROJECT_NAME} CXX C)
cmake_minimum_required(VERSION 2.8)

# Avoid source tree pollution
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
	message(FATAL_ERROR "In-source builds are not permitted. Make a separate folder for building:\nmkdir build; cd build; cmake ..\nBefore that, remove the files already created:\nrm -rf CMakeCache.txt CMakeFiles")
endif(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)

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

option(TERRAIN_USE_SHADER "Enable terrain shaders (requires shiny library and the shader files in ../media)" TRUE)

if (${TERRAIN_USE_SHADER})
    add_definitions(-DTERRAIN_USE_SHADER=1)
else()
    add_definitions(-DTERRAIN_USE_SHADER=0)
endif()

add_subdirectory (terrain)
list(APPEND LIBS "large-terrain")

# Boost
find_package(Boost COMPONENTS system thread filesystem wave REQUIRED) # Specify the required components
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
list(APPEND LIBS ${Boost_LIBRARIES})

add_subdirectory (demo)
