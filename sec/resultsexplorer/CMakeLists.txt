find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5Sql REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES main.cpp
            resultsexplorer.cpp
            resultslist.cpp)

add_executable(resultsexplorer ${SOURCES})
target_link_libraries(resultsexplorer Qt5::Widgets Qt5::PrintSupport Qt5::Network Qt5::Sql)
install (TARGETS resultsexplorer DESTINATION bin COMPONENT libonly)
