function(add_new_test target)
    add_executable(${target} ${ARGN})
    target_link_libraries(${target} robots YARP_init YARP_dev YARP_OS YARP_sig YARP_math ctrlLib iKin)

    add_test(${target} ${target})

endfunction()

add_new_test(test_nodes test_nodes.cpp)
add_new_test(test_observer test_observer.cpp)
add_new_test(test_icubsim test_icubsim.cpp)
add_new_test(test_objects test_objects.cpp)
