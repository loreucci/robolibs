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
