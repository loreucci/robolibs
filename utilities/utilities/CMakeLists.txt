set(SOURCES vector.cpp
            utilities.cpp
            derivatives.cpp
            signals.cpp
            chrono.cpp)

set(HEADERS vector.h
            utilities.h
            message.h
            derivatives.h
            signals.h
            chrono.h)


add_library(utilities SHARED ${SOURCES} ${HEADERS})
install (TARGETS utilities DESTINATION lib)
install (FILES ${HEADERS}
         DESTINATION include/utilities)
