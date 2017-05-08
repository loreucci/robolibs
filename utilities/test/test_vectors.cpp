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
