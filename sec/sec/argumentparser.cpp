#include "argumentparser.h"

#include <fstream>

#ifdef HAS_GUI
#include "launcherinterface.h"
#endif


namespace sec {


template <>
Any::Any(bool val) {
    if (val)
        value = "true";
    else
        value = "false";
}

template <>
Any::Any(std::string val) {
    value = val;
}

template<>
sec::Any::Any(const char* val) {
    value = std::string(val);
}

template<>
bool Any::getValue() const {
    bool ret;
    std::stringstream ss(value);
    ss >> std::boolalpha >> ret;
    return ret;
}

template <>
std::string Any::getValue() const {
    return value;
}


ArgumentParser argument_parser;

ArgumentParser::ArgumentParser() {
    execname = "NONAME";
}

void ArgumentParser::parseArguments(int argc, char* argv[]) {

    execname = argv[0];

    bool saveandquit = false;

    for (unsigned int i = 1; i < argc; i++) {
        std::string str = argv[i];
        if (str[0] == '-' && str[1] == '-') {
            str = str.substr(2);
            if (str == "gen-only") {
                saveandquit = true;
                continue;
            }
#ifdef HAS_GUI
            if (str == "seclauncher") {
                setLauncherMode(true);
                continue;
            }
#endif
            auto it = args.find(str);
            if (it != args.end()) {
                i++;
                if (i >= argc)
                    break;
                it->second = std::string(argv[i]);
            }
        }
    }

    if (saveandquit)
        generateFileAndQuit();
}

void ArgumentParser::saveArgsToFile(const std::string& filename) {

    std::ofstream out(filename);

    out << execname << std::endl;

    for (auto& it : args) {
        out << it.first << ";" << it.second.getValue<std::string>() << std::endl;
    }

    out.close();

}

void ArgumentParser::generateFileAndQuit() {
    saveArgsToFile();
    std::exit(0);
}

void parseArguments(int argc, char* argv[]) {
    argument_parser.parseArguments(argc, argv);
}

void saveArguments(const std::string& filename) {
    argument_parser.saveArgsToFile(filename);
}

}
