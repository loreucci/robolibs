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

#include <sec/prompter.h>

#include <iostream>
#undef NDEBUG
#include <cassert>

void printMsg(void) {

    std::cout << "test message (fun) \n";

}

class Printable {

public:
    void operator()() {
        std::cout << "test message (operator) \n";
        t = 1;
    }

    void pmember() {
        std::cout << "test message (member) \n";
        t = 2;
    }

    int t = 0;

};

void consumeString(const std::string& str) {
    std::cout << "here is your string: " << str << std::endl;
}

int main(void) {

    Printable prin;

    sec::YesNoExecutePrompt p1("Do you want to execute the function?", printMsg);
    sec::YesNoExecutePrompt p2("Do you want to execute the function?", prin);                      // no side effects, it's a copy!!!
    sec::YesNoExecutePrompt p3("Do you want to execute the function?", prin, &Printable::pmember);

    sec::StringArgExecutePrompt p4("Please insert a string:", consumeString);

    sec::ConditionalPrompt p5("Do you want to execute this other function?", p4);

    sec::Prompter prompter;
    prompter.addPrompt(p1);
    prompter.addPrompt(p2);
    prompter.addPrompt(p3);
    prompter.addPrompt(p4);
    prompter.addPrompt(p5);

    prompter.executeAll();

    std::cout << prin.t << std::endl;

    return 0;

}
