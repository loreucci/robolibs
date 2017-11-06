find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(Qt5Network REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES main.cpp
            rasterserver.cpp
            ${THIRD_PARTY}/qcustomplot/qcustomplot.cpp)

add_executable(rasterserver ${SOURCES})
target_include_directories(rasterserver PUBLIC ${THIRD_PARTY})
target_link_libraries(rasterserver Qt5::Widgets Qt5::PrintSupport Qt5::Network)
install (TARGETS rasterserver DESTINATION bin COMPONENT libonly)
