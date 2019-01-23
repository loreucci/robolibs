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
    target_link_libraries(${target} sec)

    add_test(${target} ${target})

endfunction()

add_new_test(test_commons test_commons.cpp)
add_new_test(test_nodelink test_nodelink.cpp)
add_new_test(test_filesource test_filesource.cpp)
add_new_test(test_results test_results.cpp)
add_new_test(test_arguments test_arguments.cpp)
add_new_test(test_synch test_synch.cpp)
add_new_test(test_prompter test_prompter.cpp)
add_new_test(test_events test_events.cpp)
add_new_test(test_order test_order.cpp)
add_new_test(test_clock test_clock.cpp)
add_new_test(test_trial test_trials.cpp)

if(BUILD_GUI)
    add_new_test(test_plot test_plot.cpp)
    add_new_test(test_dataexchangeserver test_dataexchangeserver.cpp)
    add_new_test(test_dataexchangeclient test_dataexchangeclient.cpp)
    add_new_test(test_launcherinterface test_launcherinterface.cpp)
endif(BUILD_GUI)
