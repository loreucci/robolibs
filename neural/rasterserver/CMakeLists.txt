# Copyright (C) 2014-2019 Lorenzo Vannucci
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

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
