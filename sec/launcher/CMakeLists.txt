find_package(Qt5Widgets REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(Qt5Network REQUIRED)
find_package(Qt5Sql REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCES main.cpp
            launcher.cpp
            dataoutwidget.cpp
            extendedspinbox.cpp)

add_executable(seclauncher ${SOURCES})
target_link_libraries(seclauncher sec Qt5::Widgets Qt5::PrintSupport Qt5::Network Qt5::Sql)
install (TARGETS seclauncher DESTINATION bin)
