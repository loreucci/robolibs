function(add_new_test target)
    add_executable(${target} ${ARGN})
    target_link_libraries(${target} utilities)

    add_test(${target} ${target})

endfunction()

#add_new_test(test_ test_.cpp)