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

#include <iostream>


class TestNode : public sec::Node {

public:
    TestNode(int i, double freq = 0.0)
        :sec::Node(freq), i(i) {}

    virtual void refreshInputs() override {}

    virtual bool connected() const override {
        return true;
    }

    virtual void execute() override {
        std::cout << i << std::endl;
    }

    virtual std::string parameters() const override {
        return "Test node " + std::to_string(i) + "\n and some other stuff that should be cut off";
    }

private:
    int i;

};


int main(void) {

    sec::setVerbose();

    sec::setDefaultFrequency(50.0);

//    std::cout << sec::isVerbose() << std::endl;

    TestNode n0(0);
    TestNode n1(1);
    TestNode n2(2);
    TestNode n3(3);
    TestNode n4(4);

    n4.setFrequency(20.0);
//    n3.setFrequency(20.0);
    n2.setFrequency(20.0);
//    n1.setFrequency(20.0);
    n0.setFrequency(20.0);

//    sec::setDefaultFrequency(50.0);

    sec::run(0.1);

    return 0;

}
