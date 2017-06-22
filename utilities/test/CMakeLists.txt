function(add_new_test target)
    add_executable(${target} ${ARGN})
    target_link_libraries(${target} utilities)

    add_test(${target} ${target})

endfunction()

add_new_test(test_vectors test_vectors.cpp)
add_new_test(test_message test_message.cpp)
add_new_test(test_signals test_signals.cpp)
add_new_test(test_derivatives test_derivatives.cpp)
add_new_test(test_integrals test_integrals.cpp)
add_new_test(test_chrono test_chrono.cpp)
