/*! \file test_chrono.cpp
    \brief Test file for the Chrono class.
*/

#include <chrono>
#include <thread>

#include "utilities/chrono.h"

#include <iostream>
#undef NDEBUG
#include <cassert>


int main(void) {

    Utils::Chrono chrono;

    // not started
    assert(chrono.getTime() < 0.0);
    assert(!chrono.isStarted());

    // measure
    chrono.start();
    std::this_thread::sleep_for(std::chrono::duration<double>(1.5));

    double t = chrono.getTime();
    assert((t>1.0) && (t<2.0));

    // multiple runs
    chrono.start();
    std::this_thread::sleep_for(std::chrono::duration<double>(1.5));

    t = chrono.getTime();
    assert((t>1.0) && (t<2.0));

    return 0;

}
