set(SOURCES utilities.cpp
            derivatives.cpp
            signals.cpp)

set(HEADERS utilities.h
            message.h
            derivatives.h
            signals.h)


add_library(utilities SHARED ${SOURCES} ${HEADERS})
install (TARGETS utilities DESTINATION lib)
install (FILES ${HEADERS}
         DESTINATION include/utilities)
