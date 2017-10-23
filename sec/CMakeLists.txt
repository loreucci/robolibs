# local includes
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# main lib
add_subdirectory(sec)

# plotting server
if(BUILD_GUI)
    add_subdirectory(plottingserver)
endif(BUILD_GUI)

# results explorer
if(BUILD_GUI)
    add_subdirectory(resultsexplorer)
endif(BUILD_GUI)

# launcher
if(BUILD_GUI)
    add_subdirectory(launcher)
endif(BUILD_GUI)

# doc
if(ENABLE_DOC)
    add_subdirectory(doc)
endif()

# testing
if(ENABLE_TEST)
    add_subdirectory(test)
endif()
