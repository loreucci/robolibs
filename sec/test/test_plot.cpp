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

#include <sec/plottingclient.h>
#include <sec/controller.h>
#include <sec/simplesources.h>
#include <sec/datalogger.h>
#include <sec/connections.h>
#include <sec/sec.h>

#include <utilities/vector.h>
#include <utilities/signals.h>

#include <cmath>


class TestVecNode : public sec::Node {

public:

    TestVecNode(double freq = 0.0)
        :sec::Node(freq) {
        Utils::Vector ret(3);
        ret[0] = -1;
        ret[1] = 2;
        ret[2] = 5;
        output = ret;
    }

    virtual void refreshInputs() override {}

    virtual bool connected() const override {
        return true;
    }

    virtual void execute() override {
        Utils::Vector ret(3);
        ret[0] = -1;
        ret[1] = 2;
        ret[2] = 5;
        output = ret;
    }

    virtual std::string parameters() const override {
        return "TestVecNode";
    }

    sec::NodeOut<Utils::Vector> output;


};



int main(void) {

    sec::setVerbose();

    Signals::Signal testsig([](double t) { return t == 0.0 ? 1.0 : 10*std::sin(3.14*2*t)/t;}, "[sin(x)/x]", 50.0);

    sec::SignalSource ss(testsig, 50.0);


    TestVecNode tvn(30.0);

    auto ssource = Signals::sin(1.0, 1.0, 0.0, 50.0);
    sec::SignalSource source(ssource, 50.0);
    sec::PlottingClient plots(50.0);

    auto ssource2 = Signals::sin(5.0, 2.0, 0.0, 50.0) + 5.0;
    sec::SignalSource source2(ssource2, 30.0);

    sec::connect(ss.output, plots, "test1");

    sec::connect(tvn.output, {0, 2}, plots, {"a", "c"}, {[](double x){return x-2;}, [](double x){return x-1;}});

    sec::connect(source2.output, plots, "test2", [](double x){return x+5.0;});
    sec::connect(source.output, plots, "test2");

    sec::main_controller.run(4.0);

}
