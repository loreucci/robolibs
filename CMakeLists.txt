# Copyright (C) 2014-2019 Lorenzo Vannucci
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

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
set(ENABLE_TEST TRUE CACHE BOOL "Testing enabling flag.")
set(ENABLE_DOC TRUE CACHE BOOL "documentation enabling flag.")

# this must be done at this level
if(ENABLE_TEST)
    enable_testing()
endif()


# flag for guis
set(BUILD_GUI TRUE CACHE BOOL "Build GUIs.")


# flags for simulators/robots
set(WITH_ROBOTS TRUE CACHE BOOL "Include support for robots library.")
if(WITH_ROBOTS)
    set(WITH_ICUB TRUE CACHE BOOL "Include iCub support.")
    set(WITH_GAZEBO TRUE CACHE BOOL "Include Gazebo simulator support.")
endif(WITH_ROBOTS)

# flags for neural simulators
set(WITH_NEURAL TRUE CACHE BOOL "Include support for neural library.")
if(WITH_NEURAL)
    set(WITH_NEST TRUE CACHE BOOL "Include NEST support.")
    set(WITH_SPINNAKER TRUE CACHE BOOL "Include SpiNNaker support.")
	if(WITH_SPINNAKER)
		if(NOT SPINNAKER_EXTERNAL_DEVICE_LIB)
			message(FATAL_ERROR "In order to compile against SpiNNaker, the variable SPINNAKER_EXTERNAL_DEVICE_LIB must be specified." )
		endif()
	endif(WITH_SPINNAKER)
endif(WITH_NEURAL)

# third party included libraries
set(THIRD_PARTY ${CMAKE_CURRENT_SOURCE_DIR}/third-party)

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

# doc
if(ENABLE_DOC)
    add_subdirectory(doc)
endif()

set(CONFIGURED_ONCE TRUE CACHE INTERNAL
    "A flag showing that CMake has configured at least once.")
