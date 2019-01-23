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

#include <sec/sec.h>
#include <sec/events.h>

#include <iostream>
#undef NDEBUG
#include <cassert>


// class case
class EventTest {

public:
    int val = 0;

    void changeVal() {
        std::cout << "event3" << std::endl;
        val = 1;
    }

};


// function case
int funVal = 0;
void funChangeVal() {
    std::cout << "event2" << std::endl;
    funVal = 1;
}


int main(void) {

    sec::EventManager manager(100.0);

    manager.addEvent([] { std::cout << "event1" << std::endl; }, 1.0);

    manager.addEvent(funChangeVal, 2.0);

    EventTest et;
    manager.addEvent(et, &EventTest::changeVal, 3.0);

    sec::run(4.0);

    assert(funVal == 1);
    assert(et.val == 1);

    return 0;

}
