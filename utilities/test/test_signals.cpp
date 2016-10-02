/*! \file test_signals.cpp
    \brief Test file for the Signals namespace.
*/

#include "utilities/signals.h"

#include <iostream>
#undef NDEBUG
#include <cassert>


int main(void) {

    Signals::sin ss(10.0, 1.0, 0.0, 100.0);
    Signals::constant c(3.0);
//    Signals::BinaryOperation so(ss, c, std::plus<double>());
    auto so = ss + c;
//    auto so2 = so + 2.0;

//    for (unsigned int i = 0; i < 500; i++) {
//        std::cout << so() << std::endl;
//    }

    Signals::ramp rs(1, 0.0, 1.0);
    Signals::rampandhold rh(1, 0.0, 3.0, 1.0);

    Signals::Switch sw(ss, c, 2.5);

    for (unsigned int i = 0; i < 500; i++) {
//        std::cout << rs() << " " << rh() << std::endl;
        std::cout << sw() << std::endl;
    }

//    std::cout << ss.to_string() << std::endl;

    return 0;

}
