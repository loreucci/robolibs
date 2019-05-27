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

#include "integrals.h"


using namespace Utils;


Integral::Integral(double freq)
    :freq(freq) {}

double Integral::integrate(double x) {
    return integrate(Utils::Vector{x})[0];
}


Vector IntegralRectangle::integrate(const Vector& x) {

    unsigned int sz = x.size();

    if (integr.size() != sz)
        integr.resize(sz);

    integr = integr + 1.0/freq*x;

    return integr;

}


IntegralTrapezoidal::IntegralTrapezoidal(double freq)
    :Integral(freq) {
    first = true;
}

Utils::Vector IntegralTrapezoidal::integrate(const Utils::Vector& x) {

    unsigned int sz = x.size();

    if (integr.size() != sz)
        integr.resize(sz);
    if (prev.size() != sz)
        prev.resize(sz);

    if (first)
        first = false;
    else
        integr = integr + 1.0/freq*(x+prev)/2.0;

    prev = x;

    return integr;

}
