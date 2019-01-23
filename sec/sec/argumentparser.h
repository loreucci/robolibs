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

#ifndef ARGUMENTPARSER_H
#define ARGUMENTPARSER_H

#include <string>
#include <sstream>
#include <unordered_map>

#include <utilities/utilities.h>


namespace sec {

// waiting for C++17 to replace this
class Any {

public:
    Any() {}

    template <typename T>
    Any(T val) {
        value = Utils::make_string(val);
    }

    template <typename T>
    T getValue() const{
        T ret;
        std::stringstream ss(value);
        ss >> ret;
        return ret;
    }

protected:
    std::string value;

};

template <>
Any::Any(bool val);

template <>
Any::Any(std::string val);

template <>
Any::Any(const char* val);

template<>
bool Any::getValue() const;

template <>
std::string Any::getValue() const;


class ArgumentParser {

public:
    ArgumentParser();

    template <typename T>
    void addArgument(const std::string& name, const T& deflt) {
        // TODO check for special chars in the name
        args[name] = deflt;
    }

    template <typename T>
    T getValue(const std::string& name) {
        return args.at(name).getValue<T>();
    }

    void parseArguments(int argc, char* argv[]);

    // this should be called after parseArguments
    void saveArgsToFile(const std::string& filename = "args.txt");

    void generateFileAndQuit();

protected:
    std::unordered_map<std::string, Any> args;
    std::string execname;
};

extern ArgumentParser argument_parser;

// convenience functions
template <typename T>
void addExpectedArgument(const std::string& name, const T& deflt) {
    argument_parser.addArgument(name, deflt);
}

template <typename T>
T getArgumentValue(const std::string& name) {
    return argument_parser.getValue<T>(name);
}

void parseArguments(int argc, char* argv[]);

void saveArguments(const std::string& filename = "args.txt");

}

#endif // ARGUMENTPARSER_H
