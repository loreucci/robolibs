add_library(utilities SHARED utilities.cpp)
install (TARGETS utilities DESTINATION lib)
install (FILES message.h utilities.h DESTINATION include/utilities)