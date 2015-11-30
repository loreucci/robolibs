set(SOURCES utilities.cpp)

set(HEADERS utilities.h
            message.h)


add_library(utilities SHARED ${SOURCES} ${HEADERS})
install (TARGETS utilities DESTINATION lib)
install (FILES ${HEADERS}
         DESTINATION include/utilities)
