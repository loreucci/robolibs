/*! \file test_integrals.cpp
    \brief Test file for the Integral class and its subclasses.
*/

#include "utilities/signals.h"
#include "utilities/integrals.h"
#include "utilities/utilities.h"

#include <iostream>
#undef NDEBUG
#include <cassert>

int main(void) {

    auto ss = Signals::sin(10.0, 1.0, Utils::PI/2.0, 100.0);
    IntegralRectangle ir(100.0);
    IntegralTrapezoidal it(100.0);

    for (unsigned int i = 0; i < 1000; i++) {
        double x = ss();
        double y1 = ir.integrate({x})[0];
        double y2 = it.integrate({x})[0];
        std::cout << x << " " << y1 << " " << y2 << std::endl;
    }

    return 0;

}
