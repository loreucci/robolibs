set(SOURCES vector.cpp
            utilities.cpp
            derivatives.cpp
            integrals.cpp
            signals.cpp
            chrono.cpp)

set(HEADERS vector.h
            utilities.h
            message.h
            derivatives.h
            integrals.h
            signals.h
            chrono.h)


add_library(utilities SHARED ${SOURCES} ${HEADERS})
target_include_directories(utilities PUBLIC ..)
install (TARGETS utilities DESTINATION lib COMPONENT libonly)
install (FILES ${HEADERS} DESTINATION include/utilities COMPONENT libonly)
