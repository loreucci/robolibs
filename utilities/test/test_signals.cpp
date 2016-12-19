/*! \file test_signals.cpp
    \brief Test file for the Signals namespace.
*/

#include "utilities/signals.h"

#include <iostream>
#undef NDEBUG
#include <cassert>


int main(void) {

    auto ss = Signals::sin(10.0, 1.0, 0.0, 100.0);
    auto c = Signals::constant(3.0);
//    Signals::BinaryOperation so(ss, c, std::plus<double>());
    auto so = 2*(ss + 3) + Signals::noise(0.0, 0.5);

//    auto so2 = 2.0 + so;
//    auto so2 = so + 2.0;

//    for (unsigned int i = 0; i < 500; i++) {
//        std::cout << so() << std::endl;
//    }

//    Signals::ramp rs(1, 0.0, 1.0);
    auto rh = Signals::rampandhold(1, 0.0, 3.0, 1.0, 100.0);

    auto sw = Signals::Switch(ss, c, 2.5);

//    auto so = 4 + ss + c + c;

//    std::cout << "prova" << std::endl;
    for (unsigned int i = 0; i < 500; i++) {
//        std::cout << rs() << " " << rh() << std::endl;
        std::cout << so() << std::endl;
    }

//    std::cout << ss.to_string() << std::endl;

    return 0;

}
