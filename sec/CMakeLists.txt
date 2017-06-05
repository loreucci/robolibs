cmake_minimum_required (VERSION 2.8 FATAL_ERROR)

project ("sec")


if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    set(cpp11 "-std=c++11")
endif()
if (NOT CONFIGURED_ONCE)
    set(CMAKE_CXX_FLAGS "${cpp11}"
        CACHE STRING "Flags used by the compiler during all build types." FORCE)
endif()

# enable testing and documentation by default
set(ENABLE_TEST TRUE
    CACHE BOOL "Testing enabling flag.")
set(ENABLE_DOC TRUE
    CACHE BOOL "documentation enabling flag.")

# robolibs folder
set(ROBOLIBS_PATH "~/usr" CACHE STRING "Folder for robolib libraries.")
get_filename_component (ROBOLIBS_PATH ${ROBOLIBS_PATH} ABSOLUTE)
include_directories ("${ROBOLIBS_PATH}/include")
link_directories ("${ROBOLIBS_PATH}/lib")

# flag for guis
set(BUILD_GUI "ON" CACHE STRING "Build GUIs.")

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
    enable_testing()
    add_subdirectory(test)
endif()

set(CONFIGURED_ONCE TRUE CACHE INTERNAL
    "A flag showing that CMake has configured at least once.")
