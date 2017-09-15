find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(Qt5Network REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES main.cpp
            plottingserver.cpp
            qcustomplot.cpp)

add_executable(plottingserver ${SOURCES})
target_link_libraries(plottingserver Qt5::Widgets Qt5::PrintSupport Qt5::Network)
install (TARGETS plottingserver DESTINATION bin COMPONENT libonly)
