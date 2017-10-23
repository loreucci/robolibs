#include "flags.h"

namespace sec {

std::unordered_map<std::string, bool> standard_flags = {
    {"verbose", false}
};


void setVerbose(bool v) {
    standard_flags["verbose"] = v;
}

bool isVerbose() {
    return standard_flags["verbose"];
}

}
