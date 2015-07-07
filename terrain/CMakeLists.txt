set(SOURCE_FILES
    world.cpp
    quadtreenode.cpp
    chunk.cpp
    material.cpp
    storage.cpp
    terrainstorage.cpp
    Simplex.cpp
    defs.hpp
    buffercache.cpp
)

add_library("large-terrain" STATIC ${SOURCE_FILES})
