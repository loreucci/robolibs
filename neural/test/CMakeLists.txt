function(add_new_test target)
    add_executable(${target} ${ARGN})
    target_link_libraries(${target} utilities sec neural)

    add_test(${target} ${target})

endfunction()

add_new_test(test_spikelogger test_spikelogger.cpp)

if(BUILD_GUI)
    add_new_test(test_raster test_raster.cpp)
    add_new_test(test_spikes test_spikes.cpp)
endif(BUILD_GUI)

if(WITH_NEST)
    add_new_test(test_nest test_nest.cpp)
    target_include_directories(test_nest PUBLIC ${PYTHON_INCLUDE_DIRS})
endif(WITH_NEST)
