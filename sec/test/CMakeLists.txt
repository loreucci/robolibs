function(add_new_test target)
    add_executable(${target} ${ARGN})
    target_link_libraries(${target} sec)

    add_test(${target} ${target})

endfunction()

add_new_test(test_plot test_plot.cpp)
add_new_test(test_nodelink test_nodelink.cpp)
add_new_test(test_filesource test_filesource.cpp)

