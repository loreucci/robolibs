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

#ifndef PROMPTER_H
#define PROMPTER_H

#include <string>
#include <vector>
#include <functional>


namespace sec {


class Prompt {

public:
    Prompt(const std::string& text);

    void execute();

    bool success();

protected:
    const std::string text;
    bool _success;

    virtual void consume(const std::string& str) = 0;

    std::string promptAndRead() const;

    virtual std::pair<std::string, bool> validate(const std::string& str) const = 0;

};


// ask for y/n and then execute accordingly
class YesNoExecutePrompt : public Prompt {

public:
    using FunType = std::function<void(void)>;

    YesNoExecutePrompt(const std::string& text, FunType fun);

    template <typename T>
    YesNoExecutePrompt(const std::string& text, T& obj, void (T::*member)(void))
        :YesNoExecutePrompt(text, [&obj, member] { (obj.*member)();}) {}

protected:
    FunType fun;

    virtual void consume(const std::string& str) override;

    virtual std::pair<std::string, bool> validate(const std::string& str) const override;

};


// read a string from stdin and use it
// just checks if the string is not empty
class StringArgExecutePrompt : public Prompt {

public:
    using FunType = std::function<void(const std::string&)>;

    StringArgExecutePrompt(const std::string& text, FunType fun);

    template <typename T>
    StringArgExecutePrompt(const std::string& text, T& obj, void (T::*member)(const std::string&))
        :StringArgExecutePrompt(text, [&obj, member] { (obj.*member)();}) {}

protected:
    FunType fun;

    virtual void consume(const std::string& str) override;

    virtual std::pair<std::string, bool> validate(const std::string& str) const override;

};


// only execute the prompt if the user answer yes to the first one
class ConditionalPrompt : public YesNoExecutePrompt {

public:
    ConditionalPrompt(const std::string& text, Prompt& p);
    ConditionalPrompt(const std::string& text, Prompt* p);

};



class Prompter {

public:
    explicit Prompter(const std::vector<Prompt*>& prompts = {});

    void executeAll();

    void addPrompt(Prompt* p);
    void addPrompt(Prompt& p);

protected:
    std::vector<Prompt*> prompts;

};

}

#endif // PROMPTER_H
