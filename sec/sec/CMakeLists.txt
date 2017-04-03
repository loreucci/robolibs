set(SOURCES
    commons.cpp
    controller.cpp
    printer.cpp
    sleeper.cpp
    logger.cpp
    nodelink.cpp
    simplenodes.cpp
    source.cpp
    node.cpp
    simplesources.cpp
    synchronization.cpp)

set(HEADERS
    commons.h
    connections.h
    controller.h
    listener.h
    logger.h
    node.h
    nodelink.h
    printer.h
    simplenodes.h
    simplesources.h
    sleeper.h
    source.h
    synchronization.h)

if(BUILD_GUI)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    find_package(Qt5Core REQUIRED)
    find_package(Qt5Network REQUIRED)
    set(SOURCES ${SOURCES} plottingclient.cpp)
    set(HEADERS ${HEADERS} plottingclient.h)
endif(BUILD_GUI)

add_library(sec SHARED ${SOURCES} ${HEADERS})

target_link_libraries(sec utilities pthread)

if(BUILD_GUI)
    target_link_libraries(sec utilities Qt5::Core Qt5::Network)
endif(BUILD_GUI)

install (TARGETS sec DESTINATION lib)
install (FILES ${HEADERS} DESTINATION include/sec)
