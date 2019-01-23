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

set(SOURCES
    commons.cpp
    controller.cpp
    printer.cpp
    sleeper.cpp
    datalogger.cpp
    nodelink.cpp
    simplenodes.cpp
    source.cpp
    node.cpp
    simplesources.cpp
    synchronization.cpp
    sec.cpp
    resultscollector.cpp
    argumentparser.cpp
    prompter.cpp
    events.cpp
    flags.cpp)

set(HEADERS
    commons.h
    connections.h
    controller.h
    datalistener.h
    datalogger.h
    node.h
    nodelink.h
    printer.h
    simplenodes.h
    simplesources.h
    sleeper.h
    source.h
    synchronization.h
    sec.h
    resultscollector.h
    argumentparser.h
    prompter.h
    events.h
    flags.h)

if(BUILD_GUI)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    find_package(Qt5Core REQUIRED)
    find_package(Qt5Network REQUIRED)
    set(SOURCES ${SOURCES} plottingclient.cpp
                           socketdataexchange.cpp
                           launcherinterface.cpp
                           qtclientserverbase.cpp)
    set(HEADERS ${HEADERS} plottingclient.h
                           socketdataexchange.h
                           launcherinterface.h
                           qtclientserverbase.h)
endif(BUILD_GUI)

add_library(sec SHARED ${SOURCES} ${HEADERS})

target_include_directories(sec PUBLIC ..)

target_link_libraries(sec utilities pthread)

if(BUILD_GUI)
    target_link_libraries(sec Qt5::Core Qt5::Network)
    target_compile_definitions(sec PRIVATE HAS_GUI)
endif(BUILD_GUI)

install (TARGETS sec DESTINATION lib COMPONENT libonly)
install (FILES ${HEADERS} DESTINATION include/sec COMPONENT libonly)
