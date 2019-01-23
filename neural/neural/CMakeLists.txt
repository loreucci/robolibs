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

# commons
set(SOURCES
    spikes.cpp
    spikelogger.cpp)

set(COMMON_HEADERS
    spikes.h
    spikelogger.h)
list(APPEND HEADERS ${COMMON_HEADERS})

# SpiNNaker
if(WITH_SPINNAKER)
    set(SPINNAKER_SOURCES
        spinnaker/basedevices.cpp
        spinnaker/datainterfaces.cpp
        spinnaker/execnode.cpp)

    set(SPINNAKER_HEADERS
        spinnaker/basedevices.h
        spinnaker/datainterfaces.h
        spinnaker/execnode.h)

    list(APPEND SOURCES ${SPINNAKER_SOURCES})
    list(APPEND HEADERS ${SPINNAKER_HEADERS})
endif(WITH_SPINNAKER)

# NEST
if(WITH_NEST)
    set(NEST_SOURCES
        nest/datainterfaces.cpp
        nest/execnode.cpp
        nest/commondevices.cpp
        nest/python_commons.cpp)

    set(NEST_HEADERS
        nest/datainterfaces.h
        nest/execnode.h
        nest/commondevices.h
        nest/python_commons.h)

    list(APPEND SOURCES ${NEST_SOURCES})
    list(APPEND HEADERS ${NEST_HEADERS})
endif(WITH_NEST)

# Qt-based
if(BUILD_GUI)
set(SOURCES ${SOURCES} rasterclient.cpp)
set(COMMON_HEADERS ${COMMON_HEADERS} rasterclient.h)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Core REQUIRED)
find_package(Qt5Network REQUIRED)
endif(BUILD_GUI)

add_library(neural SHARED ${SOURCES} ${HEADERS})

target_include_directories(neural PUBLIC ..)

target_link_libraries(neural sec)

# SpiNNaker dependencies
if(WITH_SPINNAKER)
    target_include_directories(neural PUBLIC ${SPINNAKER_EXTERNAL_DEVICE_LIB})
    target_link_libraries(neural -L${SPINNAKER_EXTERNAL_DEVICE_LIB} spynnaker_external_device_lib sqlite3)
endif(WITH_SPINNAKER)

# NEST dependencies
if(WITH_NEST)
    find_package( PythonLibs 2.7 REQUIRED )
    target_include_directories(neural PUBLIC ${PYTHON_INCLUDE_DIRS})

    find_package( Boost COMPONENTS python REQUIRED )
    target_include_directories(neural PUBLIC ${Boost_INCLUDE_DIR})
    target_link_libraries( neural ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})
endif(WITH_NEST)

# Qt linking
if(BUILD_GUI)
    target_link_libraries(neural Qt5::Core Qt5::Network)
endif(BUILD_GUI)

# install
install (TARGETS neural DESTINATION lib COMPONENT libonly)
install (FILES ${COMMON_HEADERS} DESTINATION include/neural COMPONENT libonly)

# SpiNNaker folder
if(WITH_SPINNAKER)
    install (FILES ${SPINNAKER_HEADERS} DESTINATION include/neural/spinnaker COMPONENT libonly)
endif(WITH_SPINNAKER)

# NEST folder
if(WITH_NEST)
    install (FILES ${NEST_HEADERS} DESTINATION include/neural/nest COMPONENT libonly)
endif(WITH_NEST)
