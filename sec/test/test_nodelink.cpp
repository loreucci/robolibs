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

#include "sec/simplenodes.h"
#include "sec/connections.h"
#include "sec/nodelink.h"

#include <vector>

using Dummy1 = sec::OneToOneNode<double, double>;
using Dummy2 = sec::OneToOneNode<std::vector<double>, std::vector<double>>;


int main(void) {

    std::function<double(std::vector<double>)> funct = [](std::vector<double> x){return 10.0*x[0];};

    Dummy1 d1(100.0);
    Dummy2 d2(20.0);

//    sec::connect(&d1.output, &d2.input);
//    sec::connect(d2.output, d1.input, funct);
//    sec::connect(&d2.output, &d1.input, [](std::vector<double> x){return 10.0*x[0];});

    Dummy2* d3 = new Dummy2(100.0);
    Dummy2* d4 = new Dummy2(20.0);
    Dummy2* d5 = new Dummy2(30.0);
    Dummy2 d6(40.0);
    d5->runOnSingleThread();
    d6.runOnSingleThread();

    sec::connect(3.0, d1.input);

    return 0;

}
