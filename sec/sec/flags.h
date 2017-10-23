#ifndef FLAGS_H
#define FLAGS_H

#include <unordered_map>
#include <string>


namespace sec {


extern std::unordered_map<std::string, bool> standard_flags;

// --verbose
void setVerbose(bool v = true);
bool isVerbose();


}

#endif // FLAGS_H
