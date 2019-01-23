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

#include <sec/commons.h>
#include <sec/synchronization.h>

#include <iostream>
#undef NDEBUG
#include <cassert>

int main(void) {

    auto uts1 = sec::getUniqueTimeStamp();
//    std::cout << uts1 << std::endl;
    sec::synchronizer.sleep(2000.0);
    auto uts2 = sec::getUniqueTimeStamp();
//    std::cout << uts2 << std::endl;

    assert(uts1 == uts2);

    return 0;
}
