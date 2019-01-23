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
    camera.cpp
    pid.cpp)

set(COMMON_HEADERS
    camera.h
    pid.h)
list(APPEND HEADERS ${COMMON_HEADERS})

# iCub
if(WITH_ICUB)
    set(ICUB_SOURCES
        icub/commons.cpp
        icub/icubpart.cpp
        icub/icubsim.cpp
        icub/icubrobot.cpp
        icub/nodes.cpp
        icub/icubsim_utilities.cpp
        icub/sabian.cpp
        icub/cartesian.cpp
        icub/icub.cpp)

    set(ICUB_HEADERS
        icub/commons.h
        icub/icubpart.h
        icub/icubsim.h
        icub/icubrobot.h
        icub/nodes.h
        icub/icubsim_utilities.h
        icub/sabian.h
        icub/cartesian.h
        icub/icub.h)

    list(APPEND SOURCES ${ICUB_SOURCES})
    list(APPEND HEADERS ${ICUB_HEADERS})
endif(WITH_ICUB)


# iCub
if(WITH_GAZEBO)
    set(GAZEBO_SOURCES
        gazebo/gazeborunner.cpp
        gazebo/jointforcesender.cpp
        gazebo/jointsreader.cpp
        gazebo/modelspawner.cpp
        gazebo/jointscontroller.cpp)

    set(GAZEBO_HEADERS
        gazebo/gazeborunner.h
        gazebo/jointforcesender.h
        gazebo/jointsreader.h
        gazebo/modelspawner.h
        gazebo/jointscontroller.h)

    list(APPEND SOURCES ${GAZEBO_SOURCES})
    list(APPEND HEADERS ${GAZEBO_HEADERS})
endif(WITH_GAZEBO)


add_library(robots SHARED ${SOURCES} ${HEADERS})

target_include_directories(robots PUBLIC ..)

target_link_libraries(robots utilities sec)

# iCub dependencies
if(WITH_ICUB)
    target_link_libraries(robots YARP_dev YARP_init YARP_OS YARP_sig YARP_math ctrlLib iKin)
endif(WITH_ICUB)

# Gazebo dependencies
if(WITH_GAZEBO)
    find_package(catkin REQUIRED COMPONENTS gazebo_ros)
    target_include_directories(robots PUBLIC ${catkin_INCLUDE_DIRS})
    target_link_libraries(robots ${catkin_LIBRARIES})
endif(WITH_GAZEBO)

install (TARGETS robots DESTINATION lib COMPONENT libonly)
install (FILES ${COMMON_HEADERS} DESTINATION include/robots COMPONENT libonly)

# iCub folder
if(WITH_ICUB)
    install (FILES ${ICUB_HEADERS} DESTINATION include/robots/icub COMPONENT libonly)
endif(WITH_ICUB)

# Gazebo folder
if(WITH_GAZEBO)
    install (FILES ${GAZEBO_HEADERS} DESTINATION include/robots/gazebo COMPONENT libonly)
endif(WITH_GAZEBO)
