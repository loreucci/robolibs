/*! \file test_derivatives.cpp
    \brief Test file for the Derivative class.
*/

#include "utilities/signals.h"
#include "utilities/derivatives.h"

#include <iostream>
#undef NDEBUG
#include <cassert>


int main(void) {

    auto ss = Signals::sin(10.0, 1.0, 0.0, 100.0);
    Derivative d(9, 100.0);

    for (unsigned int i = 0; i < 1000; i++) {
        double x = ss();
        double y = d.derive({x})[0];
        std::cout << x << " " << y << std::endl;
    }

    return 0;

}
