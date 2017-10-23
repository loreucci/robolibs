# local includes
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# main lib
add_subdirectory(neural)

# raster server
if(BUILD_GUI)
    add_subdirectory(rasterserver)
endif(BUILD_GUI)

# doc
if(ENABLE_DOC)
    add_subdirectory(doc)
endif()

# testing
if(ENABLE_TEST)
    add_subdirectory(test)
endif()
