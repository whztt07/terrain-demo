# The project's sources
file(GLOB PROJECT_SOURCES
	*.cpp
	sdl4ogre/*.cpp
    sdl4ogre/events.h
)

# Now compile the executable
add_executable("large-terrain-demo" ${PROJECT_SOURCES})
target_link_libraries("large-terrain-demo" ${LIBS})

