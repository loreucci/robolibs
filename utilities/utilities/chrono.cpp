#include "chrono.h"

namespace Utils {

Chrono::Chrono() {}

void Chrono::start() {
    starttime = std::chrono::system_clock::now();
}

double Chrono::getTime() {
    std::chrono::duration<double> diff = std::chrono::system_clock::now()-starttime;
    return diff.count();
}

}
