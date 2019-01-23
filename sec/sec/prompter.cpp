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

#include "prompter.h"

#include <iostream>
#include <cctype>


namespace sec {


Prompt::Prompt(const std::string& text)
    :text(text) {

    _success = false;

}

void Prompt::execute() {

    std::pair<std::string, bool> val;
    do {
        std::string str = promptAndRead();
        val = validate(str);
    } while (!val.second);

    consume(val.first);

}

bool Prompt::success() {
    return _success;
}

std::string Prompt::promptAndRead() const {

    std::cout << text << " ";
    std::string ret;
    std::getline(std::cin, ret);
    return ret;

}

YesNoExecutePrompt::YesNoExecutePrompt(const std::string& text, FunType fun)
    :Prompt(text + " [y/n]"), fun(fun) {}

void YesNoExecutePrompt::consume(const std::string& str) {

    if (str == "y") {
        fun();
        _success = true;
    }

    _success = false;

}

std::pair<std::string, bool> YesNoExecutePrompt::validate(const std::string& str) const {

    // to upper
    std::string _str = str;
    for (unsigned int i = 0; i < _str.size(); i++)
        _str[i] = std::tolower(_str[i]);

    if (_str == "y" || _str == "yes")
        return std::make_pair("y", true);

    if (str == "n" || str == "no")
        return std::make_pair("n", true);

    return std::make_pair(str, false);

}


StringArgExecutePrompt::StringArgExecutePrompt(const std::string& text, StringArgExecutePrompt::FunType fun)
    :Prompt(text), fun(fun) {}

void StringArgExecutePrompt::consume(const std::string& str) {

    fun(str);
    _success = true;

}

std::pair<std::string, bool> StringArgExecutePrompt::validate(const std::string& str) const {

    if (str.empty())
        return std::make_pair(str, false);

    return std::make_pair(str, true);

}


ConditionalPrompt::ConditionalPrompt(const std::string& text, Prompt& p)
    :YesNoExecutePrompt(text, [&p] {p.execute();}) {}

ConditionalPrompt::ConditionalPrompt(const std::string& text, Prompt* p)
    :YesNoExecutePrompt(text, [p] {p->execute();}) {}




Prompter::Prompter(const std::vector<Prompt*>& prompts)
    :prompts(prompts) {}

void Prompter::executeAll() {

    for (auto p : prompts) {
        p->execute();
    }

}

void Prompter::addPrompt(Prompt* p) {
    prompts.push_back(p);
}

void Prompter::addPrompt(Prompt& p) {
    prompts.push_back(&p);
}



}
