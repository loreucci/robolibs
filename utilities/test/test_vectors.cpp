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

/*! \file test_vectors.cpp
    \brief Test file for the Vector class.
*/

#include "utilities/vector.h"

#include "utilities/utilities.h"

#include <cmath>

#include <iostream>
#undef NDEBUG
#include <cassert>

using namespace Utils;

int main(void) {

    Vector v1{1, 2, 3};
    Vector v2{4, 5, 6};
    double k = 2;

    // associativity
    assert(v1 + (v2 + v2) == (v1 + v2) + v2);

    // commutativity
    assert(v1 + v2 == v2 + v1);

    // minus
//    std::cout << make_string(v2 - v1, " ") << std::endl;
//    std::cout << make_string(v2 + (-v1), " ") << std::endl;
    assert(v2 - v1 == v2 + (-v1));

    // scalar
    assert(k * v1 == v1 * k);

    // dot
    assert(std::abs(dot(v1, v2) - 32) < 0.01);

    std::cout << rangeNormalization(v2) << std::endl;

    return 0;

}
