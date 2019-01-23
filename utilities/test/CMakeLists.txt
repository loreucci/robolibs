# Copyright (C) 2014-2019 Lorenzo Vannucci
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

function(add_new_test target)
    add_executable(${target} ${ARGN})
    target_link_libraries(${target} utilities)

    add_test(${target} ${target})

endfunction()

add_new_test(test_vectors test_vectors.cpp)
add_new_test(test_signals test_signals.cpp)
add_new_test(test_derivatives test_derivatives.cpp)
add_new_test(test_integrals test_integrals.cpp)
add_new_test(test_chrono test_chrono.cpp)
