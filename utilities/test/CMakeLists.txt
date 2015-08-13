cmake_minimum_required (VERSION 2.6)

project (TestUtilites)

set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11 -gdwarf-3" )

enable_testing ()

# robolibs folder
set(ROBOLIBS_PATH "~/usr" CACHE STRING "Folder for robolib libraries.")
get_filename_component (ROBOLIBS_PATH ${ROBOLIBS_PATH} ABSOLUTE)
include_directories ("${ROBOLIBS_PATH}/include")
link_directories ("${ROBOLIBS_PATH}/lib")

# find all cpp files
# NOT AUTOMATICALLY UPDATED WHEN NEW FILES ARE ADDED
file (GLOB APP_SOURCES ./*.cpp)

foreach (testfile ${APP_SOURCES})

	string (REPLACE ".cpp" "" testname ${testfile})
	string (REGEX REPLACE ".*/" "" testname ${testname})
	project (${testname})
	add_executable (${testname} ${testfile})
    target_link_libraries (${testname} utilities)
	add_test(${testname} ${testname})

endforeach (testfile ${APP_SOURCES})
