function(add_new_test target)
    add_executable(${target} ${ARGN})
    target_link_libraries(${target} robots)

    add_test(${target} ${target})

endfunction()

function(add_new_test_yarp target)
    add_executable(${target} ${ARGN})
    target_link_libraries(${target} robots YARP_init YARP_dev YARP_OS YARP_sig YARP_math ctrlLib iKin)

    add_test(${target} ${target})

endfunction()


if(WITH_ICUB)
    add_new_test_yarp(test_nodes test_nodes.cpp)
    add_new_test_yarp(test_observer test_observer.cpp)
    add_new_test_yarp(test_icubsim test_icubsim.cpp)
    add_new_test_yarp(test_objects test_objects.cpp)
endif(WITH_ICUB)

if(WITH_GAZEBO)
    add_new_test(test_jointscontroller test_jointscontroller.cpp)
    add_new_test(test_modelspawner test_modelspawner.cpp)
endif(WITH_GAZEBO)
