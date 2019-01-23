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

#include "pid.h"

#include <iostream>

PID::PID(double kp, double ki, double kd, double freq)
    :sec::Node(freq), kp(kp), ki(ki), kd(kd) {

    preverr = 0.0;
    erri = 0.0;

}

void PID::refreshInputs() {
    ref.refreshData();
    enc.refreshData();
}

bool PID::connected() const {
    return ref.isConnected() && enc.isConnected();
}

void PID::execute() {

    // P
    double err = ref - enc;

    // I
    erri += (preverr+err)/freq/2.0;


    // D
    double errd = (err-preverr)/freq;

    output = kp * err + ki * erri + kd * errd;

}

std::string PID::parameters() const {

    std::string ret = "PID controller with:";
    ret += "\tKp = " + std::to_string(kp);
    ret += "\tKi = " + std::to_string(ki);
    ret += "\tKd = " + std::to_string(kd);

    return ret;

}
