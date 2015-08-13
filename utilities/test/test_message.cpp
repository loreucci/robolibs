/*! \file test_message.cpp
    \brief Test file for the Message class.
*/

#include <utilities/message.h>

#include <vector>
#include <iostream>
#include <cassert>


int main(void) {

    const unsigned int sz = 10;
    const unsigned int id = 0;

    // standard construction
    Message<double, sz, id> msg1;
    assert(msg1.size == sz);

    // vector construction
    std::vector<double> v;
    for (unsigned int i = 0; i < sz; i++) {
        v.push_back(i);
    }
    Message<double, sz, id> msg2(v);
    assert(msg2.size == v.size());
    for (unsigned int i = 0; i < sz; i++) {
        assert(msg2[i] == v[i]);
    }

    // editing
    msg2[0] = msg2[0] + 1;
    assert(msg2[0] != v[0]);

    // operations
    Message<double, sz, id> msg3 = msg1 + msg2;
    msg3 = msg1 - msg2;

    // ostream operator
    std::cout << msg3 << std::endl;

    // initializer list construction
    const Message<double, sz, id> msg4({1, 2, 3, 4, 5, 6, 7, 8, 9, 10});

    // const indexing
    for (unsigned int i = 0; i < sz; i++)
        std::cout << msg4[i] << " ";
    std::cout << std::endl;
    msg3 = msg1 + msg4 + msg2;

    return 0;

}

