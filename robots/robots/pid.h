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

#ifndef PID_H
#define PID_H

#include <sec/node.h>
#include <sec/nodelink.h>


class PID : public sec::Node {

public:
    PID(double kp, double ki, double kd, double freq = 0.0);

    virtual void refreshInputs();

    virtual bool connected() const;

    virtual void execute();

    virtual std::string parameters() const;

    // inputs
    sec::NodeIn<double> ref;
    sec::NodeIn<double> enc;

    // output
    sec::NodeOut<double> output;

protected:
    double kp, ki, kd;
    double preverr, erri;

};

#endif // PID_H
