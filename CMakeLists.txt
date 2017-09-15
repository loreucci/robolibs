cmake_minimum_required (VERSION 3.0 FATAL_ERROR)

project("robolibs" VERSION 0.1.0 LANGUAGES CXX)

# C++11 flags
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


# flag for guis
set(BUILD_GUI "ON" CACHE STRING "Build GUIs.")


# flags for simulators/robots
set(WITH_ROBOTS "ON" CACHE STRING "Include support for robots library.")
if(WITH_ROBOTS)
	set(WITH_ICUB "ON" CACHE STRING "Include iCub support.")
	set(WITH_GAZEBO "ON" CACHE STRING "Include Gazebo simulator support.")
endif(WITH_ROBOTS)

# flags for neural simulators
set(WITH_NEURAL "ON" CACHE STRING "Include support for neural library.")
if(WITH_NEURAL)
	set(WITH_NEST "ON" CACHE STRING "Include NEST support.")
	set(WITH_SPINNAKER "ON" CACHE STRING "Include SpiNNaker support.")
	if(WITH_SPINNAKER)
		if(NOT SPINNAKER_EXTERNAL_DEVICE_LIB)
			message(FATAL_ERROR "In order to compile against SpiNNaker, the variable SPINNAKER_EXTERNAL_DEVICE_LIB must be specified." )
		endif()
	endif(WITH_SPINNAKER)
endif(WITH_NEURAL)


# subdirectories
add_subdirectory(utilities)
add_subdirectory(sec)
list(APPEND LIBS utilities sec)
if(WITH_ROBOTS)
	add_subdirectory(robots)
	list(APPEND LIBS robots)
endif(WITH_ROBOTS)
if(WITH_NEURAL)
	add_subdirectory(neural)
	list(APPEND LIBS neural)
endif(WITH_NEURAL)

# workaround to avoid installing catkin stuff
add_custom_target(install-libonly
  DEPENDS ${LIBS}
  COMMAND 
      "${CMAKE_COMMAND}" -DCMAKE_INSTALL_COMPONENT=libonly
      -P "${CMAKE_BINARY_DIR}/cmake_install.cmake"
)

set(CONFIGURED_ONCE TRUE CACHE INTERNAL
    "A flag showing that CMake has configured at least once.")