set(SOURCES controller.cpp
            printer.cpp
            sleeper.cpp
            logger.cpp
            simplenodes.cpp
            source.cpp
            node.cpp
            simplesources.cpp
            synchronization.cpp)

set(HEADERS connections.h
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

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5Core REQUIRED)
find_package(Qt5Network REQUIRED)
set(SOURCES ${SOURCES} plottingclient.cpp)
set(HEADERS ${HEADERS} plottingclient.h)

add_library(sec SHARED ${SOURCES} ${HEADERS})

target_link_libraries(sec Qt5::Core Qt5::Network)

install (TARGETS sec DESTINATION lib)
install (FILES ${HEADERS}
         DESTINATION include/sec)
