/*
 * Copyright (C) 2014-2019 Lorenzo Vannucci
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

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
    SimpleDerivative sd(100.0);
    SmoothDerivative d(9, 100.0);

    for (unsigned int i = 0; i < 1000; i++) {
        double x = ss();
        double y1 = sd.derive({x})[0];
        double y2 = d.derive({x})[0];
        std::cout << x << " " << y1 << " " << y2 << std::endl;
    }

    return 0;

}
